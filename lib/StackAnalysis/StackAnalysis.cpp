/// \file StackAnalysis.cpp
/// \brief Implementation of the stack analysis, which provides information
///        about function boundaries, basic block types, arguments and return
///        values.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/Pass.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Model/Binary.h"
#include "revng/StackAnalysis/StackAnalysis.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/IRHelpers.h"

#include "Cache.h"
#include "InterproceduralAnalysis.h"
#include "Intraprocedural.h"

using llvm::BasicBlock;
using llvm::Function;
using llvm::Module;
using llvm::RegisterPass;

static Logger<> ClobberedLog("clobbered");
static Logger<> StackAnalysisLog("stackanalysis");
static Logger<> CFEPLog("cfep");

using namespace llvm::cl;

namespace StackAnalysis {

const std::set<llvm::GlobalVariable *> EmptyCSVSet;

char StackAnalysis::ID = 0;

using RegisterABI = RegisterPass<StackAnalysis>;
static RegisterABI Y("abi-analysis", "ABI Analysis Pass", true, true);

static opt<std::string> ABIAnalysisOutputPath("abi-analysis-output",
                                              desc("Destination path for the "
                                                   "ABI Analysis Pass"),
                                              value_desc("path"),
                                              cat(MainCategory));

template<bool FunctionCall>
static model::RegisterState::Values
toRegisterState(RegisterArgument<FunctionCall> RA) {
  switch (RA.value()) {
  case RegisterArgument<FunctionCall>::NoOrDead:
    return model::RegisterState::NoOrDead;
  case RegisterArgument<FunctionCall>::Maybe:
    return model::RegisterState::Maybe;
  case RegisterArgument<FunctionCall>::Yes:
    return model::RegisterState::Yes;
  case RegisterArgument<FunctionCall>::Dead:
    return model::RegisterState::Dead;
  case RegisterArgument<FunctionCall>::Contradiction:
    return model::RegisterState::Contradiction;
  case RegisterArgument<FunctionCall>::No:
    return model::RegisterState::No;
  }

  revng_abort();
}

static model::RegisterState::Values toRegisterState(FunctionReturnValue RV) {
  switch (RV.value()) {
  case FunctionReturnValue::No:
    return model::RegisterState::No;
  case FunctionReturnValue::NoOrDead:
    return model::RegisterState::NoOrDead;
  case FunctionReturnValue::YesOrDead:
    return model::RegisterState::YesOrDead;
  case FunctionReturnValue::Maybe:
    return model::RegisterState::Maybe;
  case FunctionReturnValue::Contradiction:
    return model::RegisterState::Contradiction;
  }

  revng_abort();
}

static model::RegisterState::Values
toRegisterState(FunctionCallReturnValue RV) {
  switch (RV.value()) {
  case FunctionCallReturnValue::No:
    return model::RegisterState::No;
  case FunctionCallReturnValue::NoOrDead:
    return model::RegisterState::NoOrDead;
  case FunctionCallReturnValue::YesOrDead:
    return model::RegisterState::YesOrDead;
  case FunctionCallReturnValue::Yes:
    return model::RegisterState::Yes;
  case FunctionCallReturnValue::Dead:
    return model::RegisterState::Dead;
  case FunctionCallReturnValue::Maybe:
    return model::RegisterState::Maybe;
  case FunctionCallReturnValue::Contradiction:
    return model::RegisterState::Contradiction;
  }

  revng_abort();
}

void commitToModel(GeneratedCodeBasicInfo &GCBI,
                   Function *F,
                   const FunctionsSummary &Summary,
                   model::Binary &TheBinary);

void commitToModel(GeneratedCodeBasicInfo &GCBI,
                   Function *F,
                   const FunctionsSummary &Summary,
                   model::Binary &TheBinary) {
  using namespace model;

  for (const auto &[Entry, FunctionSummary] : Summary.Functions) {
    if (Entry == nullptr)
      continue;

    //
    // Initialize model::Function
    //

    // Get the entry point address
    MetaAddress EntryPC = getBasicBlockPC(Entry);
    revng_assert(EntryPC.isValid());

    // Create the function
    revng_assert(TheBinary.Functions.count(EntryPC) == 0);
    model::Function &Function = TheBinary.Functions[EntryPC];

    // Assign a name
    Function.Name = Entry->getName();
    revng_assert(Function.Name.size() != 0);

    using FT = model::FunctionType::Values;
    Function.Type = static_cast<FT>(FunctionSummary.Type);

    if (Function.Type == model::FunctionType::Fake)
      continue;

    // Populate arguments and return values
    {
      auto Inserter = Function.Registers.batch_insert();
      for (auto &[CSV, FRD] : FunctionSummary.RegisterSlots) {
        auto ID = ABIRegister::fromCSVName(CSV->getName(), GCBI.arch());
        if (ID == model::Register::Invalid)
          continue;
        FunctionABIRegister TheRegister(ID);
        TheRegister.Argument = toRegisterState(FRD.Argument);
        TheRegister.ReturnValue = toRegisterState(FRD.ReturnValue);
        Inserter.insert(TheRegister);
      }
    }

    auto MakeEdge = [](MetaAddress Destination, FunctionEdgeType::Values Type) {
      FunctionEdge *Result = nullptr;
      if (FunctionEdgeType::isCall(Type))
        Result = new CallEdge(Destination, Type);
      else
        Result = new FunctionEdge(Destination, Type);
      return UpcastablePointer<FunctionEdge>(Result);
    };

    // Handle the situation in which we found no basic blocks at all
    if (Function.Type == model::FunctionType::NoReturn
        and FunctionSummary.BasicBlocks.size() == 0) {
      auto &EntryNodeSuccessors = Function.CFG[EntryPC].Successors;
      auto Edge = MakeEdge(MetaAddress::invalid(), FunctionEdgeType::LongJmp);
      EntryNodeSuccessors.insert(Edge);
    }

    for (auto &[BB, Branch] : FunctionSummary.BasicBlocks) {
      // Remap BranchType to FunctionEdgeType
      namespace FET = FunctionEdgeType;
      FET::Values EdgeType = FET::Invalid;

      switch (Branch) {
      case BranchType::Invalid:
      case BranchType::FakeFunction:
      case BranchType::RegularFunction:
      case BranchType::NoReturnFunction:
      case BranchType::UnhandledCall:
        revng_abort();
        break;

      case BranchType::InstructionLocalCFG:
        EdgeType = FET::Invalid;
        break;

      case BranchType::FunctionLocalCFG:
        EdgeType = FET::DirectBranch;
        break;

      case BranchType::FakeFunctionCall:
        EdgeType = FET::FakeFunctionCall;
        break;

      case BranchType::FakeFunctionReturn:
        EdgeType = FET::FakeFunctionReturn;
        break;

      case BranchType::HandledCall:
        EdgeType = FET::FunctionCall;
        break;

      case BranchType::IndirectCall:
        EdgeType = FET::IndirectCall;
        break;

      case BranchType::Return:
        EdgeType = FET::Return;
        break;

      case BranchType::BrokenReturn:
        EdgeType = FET::BrokenReturn;
        break;

      case BranchType::IndirectTailCall:
        EdgeType = FET::IndirectTailCall;
        break;

      case BranchType::LongJmp:
        EdgeType = FET::LongJmp;
        break;

      case BranchType::Killer:
        EdgeType = FET::Killer;
        break;

      case BranchType::Unreachable:
        EdgeType = FET::Unreachable;
        break;
      }

      if (EdgeType == FET::Invalid)
        continue;

      bool IsCall = FunctionEdgeType::isCall(EdgeType);

      // Identify Source address
      auto [Source, Size] = getPC(BB->getTerminator());
      Source += Size;
      revng_assert(Source.isValid());

      // Identify Destination address
      llvm::BasicBlock *JumpTargetBB = GCBI.getJumpTargetBlock(BB);
      MetaAddress JumpTargetAddress = GCBI.getPCFromNewPC(JumpTargetBB);
      model::BasicBlock &CurrentBlock = Function.CFG[JumpTargetAddress];
      CurrentBlock.End = Source;
      CurrentBlock.Name = JumpTargetBB->getName();
      auto SuccessorsInserter = CurrentBlock.Successors.batch_insert();

      if (EdgeType == FET::DirectBranch) {
        // Handle direct branch
        auto Successors = GCBI.getSuccessors(BB);
        for (const MetaAddress &Destination : Successors.Addresses)
          SuccessorsInserter.insert(MakeEdge(Destination, EdgeType));

      } else if (EdgeType == FET::FakeFunctionReturn) {
        // Handle fake function return
        auto [First, Last] = FunctionSummary.FakeReturns.equal_range(BB);
        revng_assert(First != Last);
        for (const auto &[_, Destination] : make_range(First, Last))
          SuccessorsInserter.insert(MakeEdge(Destination, EdgeType));

      } else if (IsCall) {
        // Handle call
        llvm::BasicBlock *Successor = BB->getSingleSuccessor();
        MetaAddress Destination = MetaAddress::invalid();
        if (Successor != nullptr)
          Destination = getBasicBlockPC(Successor);

        // Record the edge in the CFG
        auto TempEdge = MakeEdge(Destination, EdgeType);
        const auto &Result = SuccessorsInserter.insert(TempEdge);
        auto *Edge = llvm::cast<CallEdge>(Result.get());

        bool Found = false;
        for (const FunctionsSummary::CallSiteDescription &CSD :
             FunctionSummary.CallSites) {
          if (not CSD.Call->isTerminator() or CSD.Call->getParent() != BB)
            continue;

          revng_assert(not Found);
          Found = true;
          auto Inserter = Edge->Registers.batch_insert();
          for (auto &[CSV, FCRD] : CSD.RegisterSlots) {
            auto ID = ABIRegister::fromCSVName(CSV->getName(), GCBI.arch());
            if (ID == model::Register::Invalid)
              continue;
            FunctionABIRegister TheRegister(ID);
            TheRegister.Argument = toRegisterState(FCRD.Argument);
            TheRegister.ReturnValue = toRegisterState(FCRD.ReturnValue);
            Inserter.insert(TheRegister);
          }
        }
        revng_assert(Found);

      } else {
        // Handle other successors
        llvm::BasicBlock *Successor = BB->getSingleSuccessor();
        MetaAddress Destination = MetaAddress::invalid();
        if (Successor != nullptr)
          Destination = getBasicBlockPC(Successor);

        // Record the edge in the CFG
        SuccessorsInserter.insert(MakeEdge(Destination, EdgeType));
      }
    }
  }

  revng_check(TheBinary.verify());
}

bool StackAnalysis::runOnModule(Module &M) {
  Function &F = *M.getFunction("root");

  revng_log(PassesLog, "Starting StackAnalysis");

  auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();

  auto &LMP = getAnalysis<LoadModelWrapperPass>().get();

  // The stack analysis works function-wise. We consider two sets of functions:
  // first (Force == true) those that are highly likely to be real functions
  // (i.e., they have a direct call) and then (Force == false) all the remaining
  // candidates whose entry point is not included in any function of the first
  // set.

  struct CFEP {
    CFEP(BasicBlock *Entry, bool Force) : Entry(Entry), Force(Force) {}

    BasicBlock *Entry;
    bool Force;
  };
  std::vector<CFEP> Functions;

  // Register all the Candidate Function Entry Points
  for (BasicBlock &BB : F) {

    if (GCBI.getType(&BB) != BlockType::JumpTargetBlock)
      continue;

    uint32_t Reasons = GCBI.getJTReasons(&BB);
    bool IsFunctionSymbol = hasReason(Reasons, JTReason::FunctionSymbol);
    bool IsCallee = hasReason(Reasons, JTReason::Callee);
    bool IsUnusedGlobalData = hasReason(Reasons, JTReason::UnusedGlobalData);
    bool IsMemoryStore = hasReason(Reasons, JTReason::MemoryStore);
    bool IsPCStore = hasReason(Reasons, JTReason::PCStore);
    bool IsReturnAddress = hasReason(Reasons, JTReason::ReturnAddress);
    bool IsLoadAddress = hasReason(Reasons, JTReason::LoadAddress);

    if (IsFunctionSymbol or IsCallee) {
      // Called addresses are a strong hint
      Functions.emplace_back(&BB, true);
    } else if (not IsLoadAddress
               and (IsUnusedGlobalData
                    || (IsMemoryStore and not IsPCStore
                        and not IsReturnAddress))) {
      // TODO: keep IsReturnAddress?
      // Consider addresses found in global data that have not been used or
      // addresses that are not return addresses and do not end up in the PC
      // directly.
      Functions.emplace_back(&BB, false);
    }
  }

  for (CFEP &Function : Functions) {
    revng_log(CFEPLog,
              getName(Function.Entry) << (Function.Force ? " (forced)" : ""));
  }

  // Initialize the cache where all the results will be accumulated
  Cache TheCache(&F, &GCBI);

  // Pool where the final results will be collected
  ResultsPool Results;

  // First analyze all the `Force`d functions (i.e., with an explicit direct
  // call)
  for (CFEP &Function : Functions) {
    if (Function.Force) {
      auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();
      InterproceduralAnalysis SA(TheCache, GCBI);
      SA.run(Function.Entry, Results);
    }
  }

  // Now analyze all the remaining candidates which are not already part of
  // another function
  std::set<BasicBlock *> Visited = Results.visitedBlocks();
  for (CFEP &Function : Functions) {
    if (not Function.Force and Visited.count(Function.Entry) == 0) {
      auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();
      InterproceduralAnalysis SA(TheCache, GCBI);
      SA.run(Function.Entry, Results);
    }
  }

  for (CFEP &Function : Functions) {
    using IFS = IntraproceduralFunctionSummary;
    BasicBlock *Entry = Function.Entry;
    llvm::Optional<const IFS *> Cached = TheCache.get(Entry);
    revng_assert(Cached or TheCache.isFakeFunction(Entry));

    // Has this function been analyzed already? If so, only now we register it
    // in the ResultsPool.
    FunctionType::Values Type;
    if (TheCache.isFakeFunction(Entry))
      Type = FunctionType::Fake;
    else if (TheCache.isNoReturnFunction(Entry))
      Type = FunctionType::NoReturn;
    else
      Type = FunctionType::Regular;

    // Regular functions need to be composed by at least a basic block
    if (Cached) {
      const IFS *Summary = *Cached;
      if (Type == FunctionType::Regular)
        revng_assert(Summary->BranchesType.size() != 0);

      Results.registerFunction(Entry, Type, Summary);
    } else {
      Results.registerFunction(Entry, Type, nullptr);
    }
  }

  GrandResult = Results.finalize(&M, &TheCache);

  if (ClobberedLog.isEnabled()) {
    for (auto &P : GrandResult.Functions) {
      ClobberedLog << getName(P.first) << ":";
      for (const llvm::GlobalVariable *CSV : P.second.ClobberedRegisters)
        ClobberedLog << " " << CSV->getName().data();
      ClobberedLog << DoLog;
    }
  }

  if (StackAnalysisLog.isEnabled()) {
    std::stringstream Output;
    GrandResult.dump(&M, Output);
    TextRepresentation = Output.str();
    revng_log(StackAnalysisLog, TextRepresentation);
  }

  revng_log(PassesLog, "Ending StackAnalysis");

  if (ABIAnalysisOutputPath.getNumOccurrences() == 1) {
    std::ofstream Output;
    serialize(pathToStream(ABIAnalysisOutputPath, Output));
  }

  commitToModel(GCBI, &F, GrandResult, *LMP.getWriteableModel());

  return false;
}

void StackAnalysis::serializeMetadata(Function &F,
                                      GeneratedCodeBasicInfo &GCBI) {
  using namespace llvm;

  const FunctionsSummary &Summary = GrandResult;

  LLVMContext &Context = getContext(&F);
  QuickMetadata QMD(Context);

  // Temporary data structure so we can set all the `revng.func.member.of` in a
  // single shot at the end
  std::map<Instruction *, std::vector<Metadata *>> MemberOf;

  // Loop over all the detected functions
  for (const auto &P : Summary.Functions) {
    BasicBlock *Entry = P.first;
    const FunctionsSummary::FunctionDescription &Function = P.second;

    if (Entry == nullptr or Function.BasicBlocks.size() == 0)
      continue;

    MetaAddress EntryPC = getBasicBlockPC(Entry);

    //
    // Add `revng.func.entry`:
    // {
    //   name,
    //   address,
    //   type,
    //   { clobbered csv, ... },
    //   { { csv, argument, return value }, ... }
    // }
    //
    auto TypeMD = QMD.get(FunctionType::getName(Function.Type));

    // Clobbered registers metadata
    std::vector<Metadata *> ClobberedMDs;
    for (GlobalVariable *ClobberedCSV : Function.ClobberedRegisters) {
      if (not GCBI.isServiceRegister(ClobberedCSV))
        ClobberedMDs.push_back(QMD.get(ClobberedCSV));
    }

    // Register slots metadata
    std::vector<Metadata *> SlotMDs;
    for (auto &P : Function.RegisterSlots) {
      if (GCBI.isServiceRegister(P.first))
        continue;

      auto *CSV = QMD.get(P.first);
      auto *Argument = QMD.get(P.second.Argument.valueName());
      auto *ReturnValue = QMD.get(P.second.ReturnValue.valueName());
      SlotMDs.push_back(QMD.tuple({ CSV, Argument, ReturnValue }));
    }

    // Create revng.func.entry metadata
    MDTuple *FunctionMD = QMD.tuple({ QMD.get(getName(Entry)),
                                      QMD.get(GCBI.toConstant(EntryPC)),
                                      TypeMD,
                                      QMD.tuple(ClobberedMDs),
                                      QMD.tuple(SlotMDs) });
    Entry->getTerminator()->setMetadata("revng.func.entry", FunctionMD);

    //
    // Create func.call
    //
    for (const FunctionsSummary::CallSiteDescription &CallSite :
         Function.CallSites) {
      Instruction *Call = CallSite.Call;

      // Register slots metadata
      std::vector<Metadata *> SlotMDs;
      for (auto &P : CallSite.RegisterSlots) {
        if (GCBI.isServiceRegister(P.first))
          continue;

        auto *CSV = QMD.get(P.first);
        auto *Argument = QMD.get(P.second.Argument.valueName());
        auto *ReturnValue = QMD.get(P.second.ReturnValue.valueName());
        SlotMDs.push_back(QMD.tuple({ CSV, Argument, ReturnValue }));
      }

      Call->setMetadata("func.call", QMD.tuple(QMD.tuple(SlotMDs)));
    }

    //
    // Create revng.func.member.of
    //

    // Loop over all the basic blocks composing the function
    for (const auto &P : Function.BasicBlocks) {
      BasicBlock *BB = P.first;
      BranchType::Values Type = P.second;

      auto *Pair = QMD.tuple({ FunctionMD, QMD.get(getName(Type)) });

      // Register that this block is associated to this function
      MemberOf[BB->getTerminator()].push_back(Pair);
    }
  }

  // Apply `revng.func.member.of`
  for (auto &P : MemberOf)
    P.first->setMetadata("revng.func.member.of", QMD.tuple(P.second));
}

} // namespace StackAnalysis
