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

#include "llvm/IR/Function.h"
#include "llvm/Pass.h"

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

template<>
char StackAnalysis<true>::ID = 0;

namespace {
const char *Name = "Stack Analysis Pass";
static RegisterPass<StackAnalysis<false>> X("stack-analysis", Name, true, true);

static opt<std::string> StackAnalysisOutputPath("stack-analysis-output",
                                                desc("Destination path for the "
                                                     "Static Analysis Pass"),
                                                value_desc("path"),
                                                cat(MainCategory));

static opt<bool> UseFunctionSymbols("use-function-symbols",
                                    desc("Use function symbols in CFEP "
                                         "election"),
                                    cat(MainCategory),
                                    init(false));

} // namespace

template<>
char StackAnalysis<false>::ID = 0;

using RegisterABI = RegisterPass<StackAnalysis<true>>;
static RegisterABI Y("abi-analysis", "ABI Analysis Pass", true, true);

static opt<std::string> ABIAnalysisOutputPath("abi-analysis-output",
                                              desc("Destination path for the "
                                                   "ABI Analysis Pass"),
                                              value_desc("path"),
                                              cat(MainCategory));

template<bool AnalyzeABI>
bool StackAnalysis<AnalyzeABI>::runOnModule(Module &M) {
  Function &F = *M.getFunction("root");

  revng_log(PassesLog, "Starting StackAnalysis");

  auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();

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

    if (UseFunctionSymbols) {

      // Rely only on debug symbols to elect CFEPs.
      if (IsFunctionSymbol) {
        Functions.emplace_back(&BB, true);
      }
    } else {

      // Do the classic stuff.
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
      InterproceduralAnalysis SA(TheCache, GCBI, AnalyzeABI);
      SA.run(Function.Entry, Results);
    }
  }

  // Now analyze all the remaining candidates which are not already part of
  // another function
  std::set<BasicBlock *> Visited = Results.visitedBlocks();
  for (CFEP &Function : Functions) {
    if (not Function.Force and Visited.count(Function.Entry) == 0) {
      auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();
      InterproceduralAnalysis SA(TheCache, GCBI, AnalyzeABI);
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

  std::stringstream Output;
  GrandResult = Results.finalize(&M, &TheCache);
  GrandResult.dump(&M, Output);
  TextRepresentation = Output.str();

  if (ClobberedLog.isEnabled()) {
    for (auto &P : GrandResult.Functions) {
      ClobberedLog << getName(P.first) << ":";
      for (const llvm::GlobalVariable *CSV : P.second.ClobberedRegisters)
        ClobberedLog << " " << CSV->getName().data();
      ClobberedLog << DoLog;
    }
  }

  revng_log(StackAnalysisLog, TextRepresentation);

  revng_log(PassesLog, "Ending StackAnalysis");

  if (AnalyzeABI and ABIAnalysisOutputPath.getNumOccurrences() == 1) {
    std::ofstream Output;
    serialize(pathToStream(ABIAnalysisOutputPath, Output));
  } else if (not AnalyzeABI
             and StackAnalysisOutputPath.getNumOccurrences() == 1) {
    std::ofstream Output;
    serialize(pathToStream(StackAnalysisOutputPath, Output));
  }

  return false;
}

template<bool AnalyzeABI>
void StackAnalysis<AnalyzeABI>::serializeMetadata(Function &F) {
  using namespace llvm;

  const FunctionsSummary &Summary = GrandResult;

  LLVMContext &Context = getContext(&F);
  QuickMetadata QMD(Context);

  // Temporary data structure so we can set all the `revng.func.member.of` in a
  // single shot at the end
  std::map<Instruction *, std::vector<Metadata *>> MemberOf;

  auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();

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
    if (AnalyzeABI) {
      for (auto &P : Function.RegisterSlots) {
        if (GCBI.isServiceRegister(P.first))
          continue;

        auto *CSV = QMD.get(P.first);
        auto *Argument = QMD.get(P.second.Argument.valueName());
        auto *ReturnValue = QMD.get(P.second.ReturnValue.valueName());
        SlotMDs.push_back(QMD.tuple({ CSV, Argument, ReturnValue }));
      }
    }

    // Create revng.func.entry metadata
    MDTuple *FunctionMD = QMD.tuple({ QMD.get(getName(Entry)),
                                      QMD.get(GCBI.toConstant(EntryPC)),
                                      TypeMD,
                                      QMD.tuple(ClobberedMDs),
                                      QMD.tuple(SlotMDs) });
    Entry->getTerminator()->setMetadata("revng.func.entry", FunctionMD);

    if (AnalyzeABI) {
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

template void StackAnalysis<true>::serializeMetadata(Function &F);
template void StackAnalysis<false>::serializeMetadata(Function &F);

} // namespace StackAnalysis
