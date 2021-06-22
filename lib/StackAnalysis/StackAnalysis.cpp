/// \file StackAnalysis.cpp
/// \brief Implementation of the stack analysis, which provides information
///        about function boundaries, basic block types, arguments and return
///        values.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <climits>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Analysis/BasicAliasAnalysis.h"
#include "llvm/Analysis/ScopedNoAliasAA.h"
#include "llvm/CodeGen/UnreachableBlockElim.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar/EarlyCSE.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include "llvm/Transforms/Scalar/JumpThreading.h"
#include "llvm/Transforms/Scalar/MergedLoadStoreMotion.h"
#include "llvm/Transforms/Scalar/SROA.h"
#include "llvm/Transforms/Scalar/SimplifyCFG.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/CodeExtractor.h"
#include "llvm/Transforms/Utils/Mem2Reg.h"

#include "revng/ADT/Queue.h"
#include "revng/BasicAnalyses/RemoveHelperCalls.h"
#include "revng/BasicAnalyses/RemoveNewPCCalls.h"
#include "revng/Model/Binary.h"
#include "revng/StackAnalysis/StackAnalysis.h"
#include "revng/Support/Assert.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/MetaAddress.h"

#include "Cache.h"
#include "InterproceduralAnalysis.h"
#include "Intraprocedural.h"

using llvm::BasicBlock;
using llvm::Function;
using llvm::Module;
using llvm::RegisterPass;

using FunctionEdgeTypeValue = model::FunctionEdgeType::Values;
using FunctionTypeValue = model::FunctionType::Values;
using GCBI = GeneratedCodeBasicInfo;

using namespace llvm::cl;

static Logger<> CFEPLog("cfep");
static Logger<> ClobberedLog("clobbered");
static Logger<> StackAnalysisLog("stackanalysis");

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

static std::optional<uint64_t> electFSO(const auto &MaybeReturns) {
  auto It = std::min_element(MaybeReturns.begin(),
                             MaybeReturns.end(),
                             [](const auto &LHS, const auto &RHS) {
                               return LHS.second < RHS.second;
                             });
  if (It == MaybeReturns.end())
    return {};
  return It->second;
}

template<class FunctionOracle>
FunctionSummary CFEPAnalyzer<FunctionOracle>::analyze(
  const std::vector<llvm::GlobalVariable *> &ABIRegisters,
  llvm::BasicBlock *Entry) {
  using namespace llvm;

  Instruction *Last = nullptr;
  IRBuilder<> Builder(M.getContext());
  StructType *MetaAddressTy = MetaAddress::getStruct(&M);

  // Run function boundaries detection, and retrieve the outlined function.
  Function *OutFunc = createDisposableFunction(Entry);

  // Initialize ret hook marker, needed for the ABI analyses on return values.
  for (auto &BB : *OutFunc)
    if (auto *Ret = dyn_cast<ReturnInst>(&BB.back()))
      Last = Ret;

  if (Last) {
    for (auto *Pred : predecessors(Last->getParent())) {
      auto *Term = Pred->getTerminator();
      Builder.SetInsertPoint(Term);
      auto [NewPCStruct, _] = getPC(Term);
      Builder.CreateCall(RetHookMarker,
                         { NewPCStruct.toConstant(MetaAddressTy),
                           MetaAddress::invalid().toConstant(MetaAddressTy) });
    }
  }

  // Find registers that may be target of at least one store. This will help
  // refine the final results.
  std::set<GlobalVariable *> WrittenRegisters;
  for (auto &BB : *OutFunc) {
    for (auto &I : BB) {
      if (auto *SI = dyn_cast<StoreInst>(&I)) {
        Value *Ptr = skipCasts(SI->getPointerOperand());
        if (auto *GV = dyn_cast<GlobalVariable>(Ptr))
          WrittenRegisters.insert(GV);
      }
    }
  }

  // Retrieve results of the ABI analyses.
  ABIAnalyses::ABIAnalysesResults
    ABIResults = ABIAnalyses::analyzeOutlinedFunction(OutFunc,
                                                      *GCBI,
                                                      PreHookMarker,
                                                      RetHookMarker);

  // Identify the callee-saved registers and tell if a function returns
  // properly, i.e., it jumps to the return address. To achieve this, we craft
  // the IR by loading the PC, the SP, as well as the ABI registers CSVs
  // respectively at function entry and end. We win when the subtraction between
  // their entry and end values is zero.
  Value *RA = nullptr;
  const auto *PCH = GCBI->programCounterHandler();
  Builder.SetInsertPoint(&OutFunc->getEntryBlock().front());

  auto *IntPtrTy = GCBI->spReg()->getType();
  auto *IntTy = GCBI->spReg()->getType()->getElementType();

  auto *SP = Builder.CreateLoad(GCBI->spReg());
  auto *SPPtr = Builder.CreateIntToPtr(SP, IntPtrTy);
  auto *GEPI = Builder.CreateGEP(IntTy, SPPtr, ConstantInt::get(IntTy, 0));

  switch (GCBI->arch()) {
  case Triple::arm: {
    constexpr auto *LinkRegister = "r14";
    RA = Builder.CreateLoad(M.getGlobalVariable(LinkRegister, true));
    break;
  }

  case Triple::aarch64: {
    constexpr auto *LinkRegister = "lr";
    RA = Builder.CreateLoad(M.getGlobalVariable(LinkRegister, true));
    break;
  };

  case Triple::mips:
  case Triple::mipsel: {
    constexpr auto *ReturnRegister = "ra";
    RA = Builder.CreateLoad(M.getGlobalVariable(ReturnRegister, true));
    break;
  }

  case Triple::systemz: {
    constexpr auto *ReturnRegister = "r14";
    RA = Builder.CreateLoad(M.getGlobalVariable(ReturnRegister, true));
    break;
  }

  case Triple::x86:
  case Triple::x86_64: {
    RA = Builder.CreateLoad(SPPtr);
    break;
  }

  default:
    revng_abort("Unsupported architecture");
  }

  std::array<Value *, 4> DissectedPC = PCH->dissectJumpablePC(Builder,
                                                              RA,
                                                              GCBI->arch());

  auto *PCI = MetaAddress::composeIntegerPC(Builder,
                                            DissectedPC[0],
                                            DissectedPC[1],
                                            DissectedPC[2],
                                            DissectedPC[3]);

  SmallVector<Value *, 16> CSVI;
  Type *IsRetTy = Type::getInt128Ty(Context);
  SmallVector<Type *, 16> ArgTypes = { IsRetTy, IntTy, MetaAddressTy };
  for (auto *CSR : ABIRegisters) {
    auto *V = Builder.CreateLoad(CSR);
    CSVI.emplace_back(V);
    ArgTypes.emplace_back(IntTy);
  }

  IndirectBranchInfoMarker = Function::Create(llvm::FunctionType::get(IntTy,
                                                                      ArgTypes,
                                                                      false),
                                              GlobalValue::ExternalLinkage,
                                              "indirect_branch_info",
                                              M);
  IndirectBranchInfoMarker->addFnAttr(Attribute::NoUnwind);
  IndirectBranchInfoMarker->addFnAttr(Attribute::NoReturn);

  SmallVector<Instruction *, 4> Returns;

  if (Last) {
    for (auto *Pred : predecessors(Last->getParent())) {
      auto *Term = Pred->getTerminator();
      Builder.SetInsertPoint(Term);

      auto *PCE = PCH->composeIntegerPC(Builder);

      SmallVector<Value *, 16> CSVE;
      for (auto *CSR : ABIRegisters) {
        auto *V = Builder.CreateLoad(CSR);
        CSVE.emplace_back(V);
      }

      SP = Builder.CreateLoad(GCBI->spReg());
      SPPtr = Builder.CreateIntToPtr(SP, IntPtrTy);
      auto *GEPE = Builder.CreateGEP(IntTy, SPPtr, ConstantInt::get(IntTy, 0));

      auto *SPI = Builder.CreatePtrToInt(GEPI, IntTy);
      auto *SPE = Builder.CreatePtrToInt(GEPE, IntTy);

      // Compute the difference between the return address and the program
      // counter.
      auto *IsRet = Builder.CreateSub(PCE, PCI);

      // Compute the difference between the stack pointer values to evaluate the
      // stack height, and the difference between the CSR values as well.
      // Functions leaving the stack pointer higher than it was at function
      // entry (i.e., in an irregular state) are marked as fake functions.
      auto *SPDiff = Builder.CreateSub(SPE, SPI);

      // Save the MetaAddress of the final jump target.
      auto NewPCJT = GCBI->getJumpTarget(Term->getParent());
      GCBI->removeJTBlockFromDT(Term->getParent());
      revng_assert(NewPCJT.isValid());

      SmallVector<Value *, 16> ArgValues = {
        IsRet, SPDiff, NewPCJT.toConstant(MetaAddressTy)
      };
      for (unsigned I = 0; I < CSVI.size(); ++I) {
        auto *V = Builder.CreateSub(CSVE[I], CSVI[I]);
        ArgValues.emplace_back(V);
      }

      // Is jumping to the return address? What is the stack height?
      // What are the callee-saved registers? Where do we come from?
      auto *Call = Builder.CreateCall(IndirectBranchInfoMarker, ArgValues);
      Builder.CreateUnreachable();

      Returns.emplace_back(Term);
    }
  }

  for (auto *I : Returns)
    I->eraseFromParent();

  // Store the values of the 4 PCs in order to have them constant-folded away.
  for (auto &BB : *OutFunc) {
    for (auto &I : BB) {
      if (auto *Call = getCallTo(&I, "newpc")) {
        MetaAddress NewPC = GCBI::getPCFromNewPC(Call);
        Builder.SetInsertPoint(Call);
        PCH->setPC(Builder, NewPC);
      }
    }
  }

  // Some of the LLVM passes used later by the pipeline search for cut-offs,
  // meaning that further computation may not be done when they are reached;
  // making some optimizations opportunities missed. Thus, we change the
  // thresholds involved (e.g., the maximum value that MemorySSA uses to take
  // into account stores/phis) to have initial unbounded value.
  StringMap<cl::Option *> &Options = cl::getRegisteredOptions();
  unsigned MemSSACheckLimitDefault = getOption<unsigned>(Options,
                                                         "memssa-check-limit")
                                       ->getValue();
  unsigned MemDepBlockScanLimitDefault = getOption<unsigned>(Options,
                                                             "memdep-block-"
                                                             "scan-limit")
                                           ->getValue();

  getOption<unsigned>(Options, "memssa-check-limit")->setInitialValue(UINT_MAX);
  getOption<unsigned>(Options, "memdep-block-scan-limit")
    ->setInitialValue(UINT_MAX);

  // Run optimization passes over the disposable function.
  {
    FunctionPassManager FPM;

    // First stage: simplify the IR, promote the CSVs to local variables,
    // compute subexpressions elimination and resolve redundant expressions in
    // order to compute the stack height.
    FPM.addPass(RemoveNewPCCallsPass());
    FPM.addPass(RemoveHelperCallsPass());
    FPM.addPass(PromoteGlobalToLocalPass());
    FPM.addPass(SimplifyCFGPass());
    FPM.addPass(SROA());
    FPM.addPass(EarlyCSEPass(true));
    FPM.addPass(JumpThreadingPass());
    FPM.addPass(UnreachableBlockElimPass());
    FPM.addPass(InstCombinePass(true));
    FPM.addPass(EarlyCSEPass(true));
    FPM.addPass(SimplifyCFGPass());
    FPM.addPass(MergedLoadStoreMotionPass());
    FPM.addPass(GVN());

    // Second stage: add alias analysis info and canonicalize i2p + add into
    // geps. Since the IR may change remarkably, another round of passes is
    // necessary to take more optimization opportunities.
    FPM.addPass(SegregateDirectStackAccessesPass());
    FPM.addPass(EarlyCSEPass(true));
    FPM.addPass(InstCombinePass(true));
    FPM.addPass(GVN());

    // Third stage: if enabled, serialize the results and dump the functions on
    // disk with the alias information included as comments.
    FPM.addPass(IndirectBranchInfoPrinterPass());
    FPM.addPass(AAWriterPass());

    FunctionAnalysisManager FAM;
    FAM.registerPass([] {
      AAManager AA;
      AA.registerFunctionAnalysis<BasicAA>();
      AA.registerFunctionAnalysis<ScopedNoAliasAA>();

      return AA;
    });

    ModuleAnalysisManager MAM;
    MAM.registerPass([&] { return GeneratedCodeBasicInfoAnalysis(); });
    FAM.registerPass([&] { return GeneratedCodeBasicInfoAnalysis(); });
    FAM.registerPass([&] { return ModuleAnalysisManagerFunctionProxy(MAM); });

    PassBuilder PB;
    PB.registerFunctionAnalyses(FAM);
    PB.registerModuleAnalyses(MAM);

    FPM.run(*OutFunc, FAM);
  }

  // Restore value back.
  getOption<unsigned>(Options, "memssa-check-limit")
    ->setInitialValue(MemSSACheckLimitDefault);
  getOption<unsigned>(Options, "memdep-block-scan-limit")
    ->setInitialValue(MemDepBlockScanLimitDefault);

  FunctionSummary FunctionInfo = milkResults(ABIRegisters,
                                             ABIResults,
                                             WrittenRegisters,
                                             OutFunc);

  // Is the disposable function a FakeFunction? If so, return a
  // copy of the unoptimized disposable function.
  if (FunctionInfo.Type == FunctionTypeValue::Fake)
    FunctionInfo.FakeFunction = createDisposableFunction(Entry);

  // Discard it, no longer needed.
  throwDisposableFunction(OutFunc);
  return FunctionInfo;
}

template<class FunctionOracle>
FunctionSummary CFEPAnalyzer<FunctionOracle>::milkResults(
  const std::vector<llvm::GlobalVariable *> &ABIRegisters,
  ABIAnalyses::ABIAnalysesResults &ABIResults,
  const std::set<llvm::GlobalVariable *> &WrittenRegisters,
  llvm::Function *F) {
  using namespace llvm;

  SmallVector<std::pair<CallInst *, uint64_t>, 4> MaybeReturns;
  SmallSet<std::pair<CallInst *, FunctionEdgeTypeValue>, 4> Result;
  std::set<GlobalVariable *> CalleeSavedRegs;
  std::set<GlobalVariable *> ClobberedRegs(ABIRegisters.begin(),
                                           ABIRegisters.end());

  for (CallBase *Call : callers(IndirectBranchInfoMarker)) {
    auto *CI = cast<CallInst>(Call);
    if (CI->getParent()->getParent() == F) {
      if (isa<ConstantInt>(CI->getArgOperand(0))) {
        if (isa<ConstantInt>(CI->getArgOperand(1))) {
          int64_t FSO = cast<ConstantInt>(CI->getArgOperand(1))->getSExtValue();
          if (FSO >= 0)
            MaybeReturns.emplace_back(CI, FSO);
          else
            Result.insert({ CI, FunctionEdgeTypeValue::BrokenReturn });
        }
      }
    }
  }

  // Elect a final stack offset.
  const auto WinFSO = electFSO(MaybeReturns);

  // Did we find a valid return instruction?
  for (const auto &E : MaybeReturns) {
    auto *CI = E.first;
    if (E.second == *WinFSO) {
      Result.insert({ E.first, FunctionEdgeTypeValue::Return });
      unsigned NumArgs = CI->getNumArgOperands();
      if (NumArgs > 3) {
        for (unsigned Idx = 3; Idx < NumArgs; ++Idx) {
          if (isa<ConstantInt>(CI->getArgOperand(Idx))) {
            if (cast<ConstantInt>(CI->getArgOperand(Idx))->getZExtValue() == 0)
              CalleeSavedRegs.insert(ABIRegisters[Idx - 3]);
          }
        }
      }
    } else {
      Result.insert({ E.first, FunctionEdgeTypeValue::BrokenReturn });
    }
  }

  // Neither a return nor a broken return?
  for (CallBase *Call : callers(IndirectBranchInfoMarker)) {
    auto *CI = cast<CallInst>(Call);
    if (CI->getParent()->getParent() == F) {
      if (isa<ConstantInt>(CI->getArgOperand(0)))
        continue;

      if (WinFSO.has_value() && (isa<ConstantInt>(CI->getArgOperand(1)))
          && (cast<ConstantInt>(CI->getArgOperand(1))->getZExtValue()
              == *WinFSO))
        Result.insert({ CI, FunctionEdgeTypeValue::IndirectTailCall });
      else
        Result.insert({ CI, FunctionEdgeTypeValue::LongJmp });
    }
  }

  bool FoundRet, FoundBrokenRet;
  FoundRet = FoundBrokenRet = false;
  for (const auto &E : Result) {
    if (E.second == FunctionEdgeTypeValue::Return)
      FoundRet = true;
    if (E.second == FunctionEdgeTypeValue::BrokenReturn)
      FoundBrokenRet = true;
  }

  FunctionTypeValue Type;
  if (FoundRet) {
    Type = FunctionTypeValue::Regular;
  } else if (FoundBrokenRet) {
    Type = FunctionTypeValue::Fake;
  } else {
    Type = FunctionTypeValue::NoReturn;
  }

  // Finally retrieve the clobbered registers and return them.
  std::erase_if(ClobberedRegs, [&](const auto &E) {
    return CalleeSavedRegs.find(E) != CalleeSavedRegs.end();
  });

  // We say that a register is callee-saved when, besides being preserved by the
  // callee, there is at least a write onto this register.
  std::set<GlobalVariable *> ActualCalleeSavedRegs;
  std::set_intersection(CalleeSavedRegs.begin(),
                        CalleeSavedRegs.end(),
                        WrittenRegisters.begin(),
                        WrittenRegisters.end(),
                        std::inserter(ActualCalleeSavedRegs,
                                      ActualCalleeSavedRegs.begin()));

  // Union between effective callee-saved registers and SP.
  std::set<GlobalVariable *> CalleeSavedAndStackPointerRegs = {
    ActualCalleeSavedRegs
  };
  CalleeSavedAndStackPointerRegs.insert(GCBI->spReg());

  // Refine ABI analyses by suppressing callee-saved and stack pointer registers
  // from the final results.
  for (const auto &Reg : CalleeSavedAndStackPointerRegs) {
    if (ABIResults.Arguments.count(Reg) != 0)
      ABIResults.Arguments[Reg] = model::RegisterState::Values::No;
  }

  for (const auto &[K, _] : ABIResults.CallSites) {
    for (const auto &Reg : CalleeSavedAndStackPointerRegs) {
      if (ABIResults.CallSites[K].Arguments.count(Reg) != 0)
        ABIResults.CallSites[K]
          .Arguments[Reg] = model::RegisterState::Values::No;

      if (ABIResults.CallSites[K].ReturnValues.count(Reg) != 0)
        ABIResults.CallSites[K]
          .ReturnValues[Reg] = model::RegisterState::Values::No;
    }
  }

  for (const auto &E : Result) {
    if (E.second != FunctionEdgeTypeValue::Return
        && E.second != FunctionEdgeTypeValue::IndirectTailCall) {
      auto PC = MetaAddress::fromConstant(E.first->getOperand(2));

      auto It = ABIResults.ReturnValues.find(PC);
      if (It != ABIResults.ReturnValues.end())
        ABIResults.ReturnValues.erase(PC);
    }
  }

  for (const auto &[K, _] : ABIResults.ReturnValues) {
    for (const auto &Reg : CalleeSavedAndStackPointerRegs)
      if (ABIResults.ReturnValues[K].count(Reg) != 0)
        ABIResults.ReturnValues[K][Reg] = model::RegisterState::Values::No;
  }

  ABIAnalyses::finalizeReturnValues(ABIResults);

  if (StackAnalysisLog.isEnabled()) {
    StackAnalysisLog << "Dumping final ABIAnalyses results for function "
                     << F->getName() << ": \n";
    ABIResults.dump();
  }

  return FunctionSummary(Type, ClobberedRegs, WinFSO, nullptr);
}

template<class FunctionOracle>
llvm::BasicBlock *
CFEPAnalyzer<FunctionOracle>::integrateFunctionCallee(llvm::BasicBlock *BB) {
  using namespace llvm;

  // In case the basic block has a call-site, the function call is
  // replaced with i) hooks that facilitate the ABI analysis ii) summary
  // of the registers clobbered by that function. The newly created basic block
  // is returned so that it can be added to the ValueToValueMap.
  BasicBlock *Split = nullptr;
  auto *Term = BB->getTerminator();
  auto *Call = getFunctionCall(Term);

  // If the function callee is null, we are dealing with an indirect call.
  auto *Next = getFunctionCallCallee(Term);

  // What is the function type of the callee?
  FunctionTypeValue Ty = Oracle.getFunctionType(Next);

  switch (Ty) {
  case FunctionTypeValue::Regular: {
    BranchInst *Br = BranchInst::Create(getFallthrough(Term));
    ReplaceInstWithInst(Term, Br);

    // Extract MetaAddress of `newpc` of the call-site and the callee.
    auto [NewPCCallSite, _] = getPC(Br);
    auto NewPCCallee = Next ? getBasicBlockPC(Next) : MetaAddress::invalid();

    // What are the registers clobbered by the callee?
    const auto &ClobberedRegs = Oracle.getRegistersClobbered(Next);
    IRBuilder<> Builder(Br);

    // Mark end of basic block with a pre-hook call.
    StructType *MetaAddressTy = MetaAddress::getStruct(&M);
    SmallVector<Value *, 2> Args = { NewPCCallSite.toConstant(MetaAddressTy),
                                     NewPCCallee.toConstant(MetaAddressTy) };
    auto *Last = Builder.CreateCall(PreHookMarker, Args);

    // Prevent the store instructions from being optimized out by storing
    // the rval of a call to an opaque function into the clobbered registers.
    OFPRegistersClobbered.addFnAttribute(Attribute::ReadOnly);
    OFPRegistersClobbered.addFnAttribute(Attribute::NoUnwind);

    for (GlobalVariable *Reg : ClobberedRegs) {
      auto *CSVTy = Reg->getType()->getPointerElementType();
      auto *RegsClobbOpqF = OFPRegistersClobbered
                              .get(BB->getParent()->getName(),
                                   CSVTy,
                                   {},
                                   "registers_clobbered");
      Builder.CreateStore(Builder.CreateCall(RegsClobbOpqF), Reg);
    }

    // Adjust back the stack pointer.
    switch (GCBI->arch()) {
    case llvm::Triple::x86:
    case llvm::Triple::x86_64: {
      auto *SPCSV = M.getGlobalVariable(GCBI->spReg()->getName(), true);
      auto *SP = Builder.CreateLoad(SPCSV);

      // What is the elected final stack offset?
      const auto &FSO = Oracle.getElectedFSO(Next);
      Value *Offset = GCBI->arch() == llvm::Triple::x86 ?
                        Builder.getInt32(*FSO) :
                        Builder.getInt64(*FSO);
      auto *Inc = Builder.CreateAdd(SP, Offset);
      Builder.CreateStore(Inc, SPCSV);
      break;
    }
    default: {
      break;
    }
    }

    // Mark end of basic block with a post-hook call.
    Builder.CreateCall(PostHookMarker, Args);

    Split = BB->splitBasicBlock(Last->getPrevNode(),
                                BB->getName() + Twine("__summary"));
    break;
  }

  case FunctionTypeValue::NoReturn: {
    CallInst *Abort = CallInst::Create(M.getFunction("abort"));
    new UnreachableInst(BB->getContext(), BB);
    ReplaceInstWithInst(Term, Abort);
    break;
  }

  case FunctionTypeValue::Fake: {
    BranchInst *Br = BranchInst::Create(getFallthrough(Term));

    // Get the fake function by its entry basic block.
    Function *FakeFunction = Oracle.getFakeFunction(Next);

    // If fake, it must have been already analyzed.
    revng_assert(FakeFunction != nullptr);

    // If possible, inline the fake function.
    auto *CI = CallInst::Create(FakeFunction, "", Term);
    InlineFunctionInfo IFI;
    bool Status = InlineFunction(*CI, IFI, nullptr, true).isSuccess();
    revng_log(StackAnalysisLog,
              "Has callee " << FakeFunction->getName() << "been inlined? "
                            << Status);

    // Still need to replace the successor with the fall-through.
    ReplaceInstWithInst(Term, Br);
    break;
  }

  default:
    revng_abort();
  }

  Call->eraseFromParent();
  return Split;
}

template<class FunctionOracle>
llvm::Function *CFEPAnalyzer<FunctionOracle>::createDisposableFunction(
  llvm::BasicBlock *Entry) {
  using namespace llvm;
  Function *Root = Entry->getParent();

  OnceQueue<BasicBlock *> Queue;
  std::vector<BasicBlock *> BlocksToClone;
  Queue.insert(Entry);

  while (!Queue.empty()) {
    BasicBlock *Current = Queue.pop();
    BlocksToClone.emplace_back(Current);

    if (isFunctionCall(Current)) {
      auto *Succ = getFallthrough(Current);
      auto *Next = getFunctionCallCallee(Current);

      if (Oracle.getFunctionType(Next) != FunctionTypeValue::NoReturn)
        Queue.insert(Succ);
      continue;
    }

    for (auto *Succ : successors(Current)) {
      if (!GCBI::isPartOfRootDispatcher(Succ))
        Queue.insert(Succ);
    }
  }

  ValueToValueMapTy VMap;
  SmallVector<BasicBlock *, 8> BlocksToExtract;

  for (const auto &BB : BlocksToClone) {
    auto *Cloned = CloneBasicBlock(BB, VMap, Twine("__cloned"), Root);

    BasicBlock *New = nullptr;
    if (isFunctionCall(Cloned)) {
      New = integrateFunctionCallee(Cloned);
      VMap[Cloned] = New;
    }

    VMap[BB] = Cloned;
    BlocksToExtract.emplace_back(Cloned);
    if (New)
      BlocksToExtract.emplace_back(New);
  }

  remapInstructionsInBlocks(BlocksToExtract, VMap);

  CodeExtractorAnalysisCache CEAC(*Root);
  Function *OutlinedFunction = CodeExtractor(BlocksToExtract)
                                 .extractCodeRegion(CEAC);

  revng_assert(OutlinedFunction != nullptr);
  revng_assert(OutlinedFunction->arg_size() == 0);
  revng_assert(OutlinedFunction->getReturnType()->isVoidTy());

  cast<Instruction>(*OutlinedFunction->user_begin())
    ->getParent()
    ->eraseFromParent();

  return OutlinedFunction;
}

template<class FunctionOracle>
void CFEPAnalyzer<FunctionOracle>::throwDisposableFunction(llvm::Function *F) {
  revng_assert(F->use_empty()
               && "Failed to remove all users of the outlined function.");

  F->eraseFromParent();
  IndirectBranchInfoMarker->eraseFromParent();
  F = nullptr;
  IndirectBranchInfoMarker = nullptr;
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

  // Initialize hook markers for subsequent ABI analysis on function calls.
  auto *FTy = llvm::FunctionType::get(llvm::Type::getVoidTy(M.getContext()),
                                      { MetaAddress::getStruct(&M),
                                        MetaAddress::getStruct(&M) },
                                      false);
  auto *PreHookMarker = Function::Create(FTy,
                                         llvm::GlobalValue::ExternalLinkage,
                                         "precall_hook",
                                         M);
  PreHookMarker->addFnAttr(llvm::Attribute::ReadOnly);
  PreHookMarker->addFnAttr(llvm::Attribute::NoUnwind);

  auto *PostHookMarker = Function::Create(FTy,
                                          llvm::GlobalValue::ExternalLinkage,
                                          "postcall_hook",
                                          M);
  PostHookMarker->addFnAttr(llvm::Attribute::ReadOnly);
  PostHookMarker->addFnAttr(llvm::Attribute::NoUnwind);

  auto *RetHookMarker = Function::Create(FTy,
                                         llvm::GlobalValue::ExternalLinkage,
                                         "retcall_hook",
                                         M);
  RetHookMarker->addFnAttr(llvm::Attribute::ReadOnly);
  RetHookMarker->addFnAttr(llvm::Attribute::NoUnwind);

  // Queue to be populated with the CFEP.
  llvm::SmallVector<BasicBlock *, 8> Worklist;
  llvm::SmallVector<BasicBlockNode *, 4> EntryBlocks;
  UniquedQueue<BasicBlockNode *> CFEPQueue;
  SmallCallGraph CG{};

  using BasicBlockToNodeMapTy = llvm::DenseMap<BasicBlock *, BasicBlockNode *>;
  BasicBlockToNodeMapTy BasicBlockNodeMap;

  // Create over-approximated call graph.
  for (CFEP &Function : Functions) {
    BasicBlockNode Node{ Function.Entry };
    auto *GraphNode = CG.addNode(Node);
    BasicBlockNodeMap[Function.Entry] = GraphNode;
  }

  for (CFEP &Function : Functions) {
    llvm::SmallSet<BasicBlock *, 8> Visited;
    auto *StartNode = BasicBlockNodeMap[Function.Entry];
    Worklist.emplace_back(Function.Entry);

    do {
      auto *Current = Worklist.pop_back_val();
      Visited.insert(Current);
      for (auto *Succ : successors(Current)) {
        if (isFunctionCall(Current)) {
          auto *Callee = getFunctionCallCallee(Current);
          // If not an indirect call, add the node to the CG.
          if (Callee) {
            auto *Node = BasicBlockNodeMap[Callee];
            StartNode->addSuccessor(Node);
          }
          auto *Next = getFallthrough(Current);
          if (!Visited.count(Next))
            Worklist.push_back(Next);
          continue;
        }

        if (!GCBI::isPartOfRootDispatcher(Succ) && !Visited.count(Succ))
          Worklist.push_back(Succ);
      }
    } while (!Worklist.empty());
  }

  for (const auto &[_, Node] : BasicBlockNodeMap) {
    if (!Node->hasPredecessors())
      EntryBlocks.emplace_back(Node);

    // Recursive?
    if (*Node->predecessors().begin() == Node)
      EntryBlocks.emplace_back(Node);
  }

  BasicBlockNode FakeNode{ nullptr };
  auto *RootNode = CG.addNode(FakeNode);
  CG.setEntryNode(RootNode);
  for (const auto &Node : EntryBlocks)
    RootNode->addSuccessor(Node);

  if (StackAnalysisLog.isEnabled()) {
    std::error_code EC;
    llvm::raw_fd_ostream Out(M.getModuleIdentifier() + "_CG.dot",
                             EC,
                             llvm::sys::fs::OF_None);
    llvm::WriteGraph(Out, &CG);
    revng_log(StackAnalysisLog, "Call graph written on disk");
    Out.close();
  }

  for (auto *Node : llvm::post_order(&CG)) {
    if (Node->hasPredecessors())
      CFEPQueue.insert(Node);
  }

  // Collect all the ABI registers.
  std::vector<llvm::GlobalVariable *> ABIRegisters;
  for (auto *CSV : GCBI.abiRegisters())
    if (CSV && !(GCBI.isSPReg(CSV)))
      ABIRegisters.emplace_back(CSV);

  std::optional<uint64_t> FSO;
  switch (GCBI.arch()) {
  case llvm::Triple::x86: {
    FSO = 4;
    break;
  }
  case llvm::Triple::x86_64: {
    FSO = 8;
    break;
  }
  default: {
    FSO = 0;
    break;
  }
  }

  using CFEPA = CFEPAnalyzer<FunctionAnalysisResults>;

  // Cache summary default-constructed for indirect calls.
  FunctionSummary DefaultSummary(model::FunctionType::Values::Regular,
                                 { ABIRegisters.begin(), ABIRegisters.end() },
                                 FSO,
                                 nullptr);
  FunctionAnalysisResults Properties(std::move(DefaultSummary));

  // Interprocedural analysis.
  while (!CFEPQueue.empty()) {
    auto *EntryNode = CFEPQueue.pop();
    revng_log(StackAnalysisLog,
              "Analyzing Entry: " << EntryNode->BB->getName());

    // Intraprocedural analysis.
    CFEPA Analyzer(M,
                   &GCBI,
                   Properties,
                   PreHookMarker,
                   PostHookMarker,
                   RetHookMarker);
    FunctionSummary Res = Analyzer.analyze(ABIRegisters, EntryNode->BB);
    bool Changed = Properties.registerFunction(EntryNode->BB, std::move(Res));

    // Need to recompute the queue?
    if (Changed) {
      UniquedQueue<BasicBlockNode *> FakeFunctionWorklist;
      FakeFunctionWorklist.insert(EntryNode);

      while (!FakeFunctionWorklist.empty()) {
        auto *Node = FakeFunctionWorklist.pop();
        for (auto *Caller : Node->predecessors()) {
          if (Caller == RootNode)
            break;

          if (!Properties.isFakeFunction(Caller->BB))
            CFEPQueue.insert(Caller);
          else
            FakeFunctionWorklist.insert(Caller);
        }
      }
    }
  }

  // Still OK?
  revng_assert(llvm::verifyModule(M, &llvm::dbgs()) == false);

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

  commitToModel(GCBI, &F, GrandResult, LMP.getWriteableModel());

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
