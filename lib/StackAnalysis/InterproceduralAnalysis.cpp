/// \file InterproceduralAnalysis.cpp
/// \brief Implementation of the interprocedural portion of the stack analysis

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <chrono>

#include "revng/Support/Statistics.h"

#include "InterproceduralAnalysis.h"

#include "Cache.h"

using llvm::BasicBlock;
using llvm::GlobalVariable;
using llvm::Instruction;
using llvm::Module;
using llvm::Optional;
using llvm::StringRef;

using time_point = std::chrono::steady_clock::time_point;
using StringIntCounter = CounterMap<std::string, uint64_t>;

Logger<> SaInterpLog("sa-interp");

/// \brief Logger for counting how much time is spent on a function
static StringIntCounter FunctionAnalysisTime("FunctionAnalysisTime");

/// \brief Logger for counting how many times a function is analyzed
static StringIntCounter FunctionAnalysisCount("FunctionAnalysisCount");

template<typename T>
static uint64_t nanoseconds(T Span) {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(Span).count();
}

namespace StackAnalysis {

void InterproceduralAnalysis::push(BasicBlock *Entry) {
  InProgressFunctions.insert(Entry);
  InProgress.emplace_back(Entry,
                          TheCache,
                          &GCBI,
                          InProgressFunctions,
                          AnalyzeABI);
  FunctionAnalysisCount.push(Entry->getName().str());
}

void InterproceduralAnalysis::run(BasicBlock *Entry, ResultsPool &Results) {
  using IFS = IntraproceduralFunctionSummary;

  revng_assert(InProgress.size() == 0);

  Optional<const IFS *> Cached = TheCache.get(Entry);

  // Has this function been analyzed already? If so, skip it.
  if (Cached)
    return;

  // Setup logger: each time we start a new intraprocedural analysis we indent
  // the output
  SaInterpLog.setIndentation(0);
  revng_log(SaInterpLog, "Running interprocedural analysis on " << Entry);

  // Push the request function in the worklist
  push(Entry);

  FunctionType::Values Type = FunctionType::Invalid;
  auto Result = Intraprocedural::Interrupt::createInvalid();

  // Loop over the worklist
  do {
    Type = FunctionType::Invalid;

    // Get an element from the stack (but don't pop yet)
    Analysis &Current = InProgress.back();

    SaInterpLog.setIndentation(InProgress.size());
    revng_log(SaInterpLog,
              "Analyzing function "
                << Current.entry() << " (size: " << Current.size()
                << " BBs, MustHit: " << Current.cacheMustHit() << ")");
    SaInterpLog.indent();

    time_point Begin = std::chrono::steady_clock::now();

    // Run/continue the intraprocedural analysis
    Result = Current.run();

    time_point End = std::chrono::steady_clock::now();
    FunctionAnalysisTime.push(Current.entry()->getName().str(),
                              nanoseconds(End - Begin));

    revng_assert(Result.requiresInterproceduralHandling());

    switch (Result.type()) {
    case BranchType::FakeFunction:
      revng_log(SaInterpLog, Current.entry() << " is fake");

      // It's a fake function, mark it as so and resume from the caller
      TheCache.markAsFake(Current.entry());

      // Set the function type in case this is the last in the worklist
      Type = FunctionType::Fake;

      // If it was recursive, pop until the recurions root (excluded, for now)
      if (const auto *Root = getRecursionRoot(Current.entry()))
        popUntil(Root);

      // Go back to the caller of the fake function.
      pop();

      // We basically evicted a function call which was supposed to hit the
      // cache. Reset the flag.
      if (InProgress.size() > 0)
        InProgress.back().resetCacheMustHit();

      break;

    case BranchType::UnhandledCall: {
      BasicBlock *Callee = Result.getCallee();
      revng_assert(Callee != nullptr);

      revng_log(SaInterpLog,
                Current.entry() << " performs an unhandled call to " << Callee);

      // Is it recursive?
      if (getRecursionRoot(Callee) != nullptr) {
        if (SaInterpLog.isEnabled()) {
          SaInterpLog << Callee << " is recursive. Call stack:";
          for (const Analysis &WorkItem : InProgress)
            SaInterpLog << " " << WorkItem.entry();
          SaInterpLog << DoLog;
        }

        // We now have to inject in the cache a temporary entry.
        TheCache.update(Callee, IFS::bottom());

        // Ensure it was inserted in the cache
        revng_assert(TheCache.get(Callee));

        // At this point the intraprocedural analysis will resume employing
        // bottom for the recursive call. Then, once the analysis is done, the
        // interprocedural part will detect that the result associated with it
        // has changed (hopefully the result won't be bottom) and will run the
        // analysis again until we're stable.
      } else {
        // Just a regular (uncached) function call, push it on the stack
        push(Callee);
      }

      break;
    }

    case BranchType::NoReturnFunction:
    case BranchType::RegularFunction: {
      const IFS &Summary = Result.getFunctionSummary();

      revng_log(SaInterpLog, "We have a summary for " << Current.entry());

      bool MustReanalyze = false;

      // Are there function calls that lead to a contradiction?
      const std::set<BasicBlock *> &Offending = Current.incoherentFunctions();
      if (Offending.size() != 0) {
        // If so, mark the called function as fake and re-analyze the caller
        for (BasicBlock *Entry : Offending) {
          revng_log(SaInterpLog,
                    Entry << " leads to contradiction, marking it as fake");

          revng_assert(Current.entry() != Entry);
          TheCache.markAsFake(Entry);
        }

        MustReanalyze = true;
      } else {
        // OK, no contradictions

        // TODO: we should probably move the cleanup earlier on
        // Perform some maintainance before recording in the cache
        IFS SummaryForCache = Summary.copy();
        SummaryForCache.FinalState.cleanup();

        // Let's register the result in the cache and check if we got any
        // changes w.r.t. to the last time we analyzed this function
        MustReanalyze = TheCache.update(Current.entry(), SummaryForCache);

        revng_assert(TheCache.get(Current.entry()));
      }

      if (SaLog.isEnabled()) {
        revng_log(SaLog, "FinalState for " << getName(Current.entry()));
        Summary.dump(getModule(Entry), SaLog);
      }

      if (MustReanalyze) {
        revng_log(SaInterpLog, "Something has changed, let's reanalyze");

        // Go back to the root of the recursion, if any
        if (const auto *Root = getRecursionRoot(Current.entry()))
          popUntil(Root);

        // Something changed, reset and re-run the analysis
        Current.initialize();

      } else {

        revng_log(SaInterpLog,
                  "No improvement over the last analysis, we're OK");

        switch (Result.type()) {
        case BranchType::NoReturnFunction:
          revng_log(SaInterpLog, Current.entry() << " doesn't return");
          TheCache.markAsNoReturn(Current.entry());
          Type = FunctionType::NoReturn;
          break;

        case BranchType::RegularFunction:
          Type = FunctionType::Regular;
          break;

        default:
          revng_abort();
        }

        // We're done here, let's go up one position in the stack
        pop();
      }

    } break;

    case BranchType::InstructionLocalCFG:
    case BranchType::FunctionLocalCFG:
    case BranchType::FakeFunctionCall:
    case BranchType::FakeFunctionReturn:
    case BranchType::HandledCall:
    case BranchType::IndirectCall:
    case BranchType::Return:
    case BranchType::BrokenReturn:
    case BranchType::IndirectTailCall:
    case BranchType::LongJmp:
    case BranchType::Killer:
    case BranchType::Unreachable:
    case BranchType::Invalid:
      revng_abort("Unexpected branch type in interprocedural analysis");
    }

  } while (InProgress.size() > 0);

  revng_assert(Type != FunctionType::Invalid);
}

void ResultsPool::mergeFunction(BasicBlock *Function,
                                const IntraproceduralFunctionSummary &Summary) {
  using FRA = FunctionRegisterArgument;
  using FRV = FunctionReturnValue;
  using FCRA = FunctionCallRegisterArgument;
  using FCRV = FunctionCallReturnValue;

  const Module *M = getModule(Function);
  size_t CSVCount = std::distance(M->global_begin(), M->global_end());

  LocallyWrittenRegisters[Function] = Summary.WrittenRegisters;

  // Merge results from the arguments analyses
  const FunctionABI &ABI = Summary.ABI;
  auto &Slots = Summary.LocalSlots;
  for (auto &Slot : Slots) {
    int32_t Offset = Slot.first.offset();
    FunctionSlot Key = { Function, Offset };

    revng_assert(Slot.first.addressSpace() == ASID::cpuID()
                 and Offset <= static_cast<int32_t>(CSVCount));

    switch (Slot.second) {
    case LocalSlotType::UsedRegister:
    case LocalSlotType::ForwardedArgument:
    case LocalSlotType::ForwardedReturnValue:
      ABI.applyResults(FunctionRegisterArguments[Key], Offset);
      ABI.applyResults(FunctionReturnValues[Key], Offset);

      // Handle forwarded arguments/return values (push rax; pop rdx)
      if (Slot.second == LocalSlotType::ForwardedArgument
          && FunctionRegisterArguments[Key].value() == FRA::Yes) {
        FunctionRegisterArguments[Key] = FRA::maybe();
      }

      if (Slot.second == LocalSlotType::ForwardedReturnValue
          && (FunctionReturnValues[Key].value() == FRV::YesOrDead)) {
        FunctionReturnValues[Key] = FRV::maybe();
      }

      break;

    case LocalSlotType::ExplicitlyCalleeSavedRegister:
      FunctionRegisterArguments[Key] = FRA::no();
      FunctionReturnValues[Key] = FRV::no();
      ExplicitlyCalleeSavedRegisters[Function].insert(Slot.first.offset());
      break;
    }

    for (auto &P : CallSites) {
      CallSite Call = P.first;

      if (!Call.belongsTo(Function))
        continue;

      FunctionCallSlot K = { Call, Offset };
      Instruction *I = Call.callInstruction();
      FunctionCall TheCall = { getFunctionCallCallee(I->getParent()), I };

      switch (Slot.second) {
      case LocalSlotType::UsedRegister:
      case LocalSlotType::ForwardedArgument:
      case LocalSlotType::ForwardedReturnValue:
      case LocalSlotType::ExplicitlyCalleeSavedRegister:

        ABI.applyResults(FunctionCallRegisterArguments[K], TheCall, Offset);
        ABI.applyResults(FunctionCallReturnValues[K], TheCall, Offset);

        // Handle forwarded arguments/return values (push rax; pop rdx)
        if (Slot.second == LocalSlotType::ForwardedArgument
            && FunctionCallRegisterArguments[K].value() == FCRA::Yes) {
          FunctionCallRegisterArguments[K] = FCRA::maybe();
        }

        if (Slot.second == LocalSlotType::ForwardedReturnValue
            && FunctionCallReturnValues[K].value() == FCRV::Yes) {
          FunctionCallReturnValues[K] = FCRV::maybe();
        }
      }
    }
  }
}

void ResultsPool::mergeBranches(BasicBlock *Function,
                                const BasicBlockTypeMap &Branches) {
  // Merge information about the branches type
  for (auto &P : Branches)
    BranchesType[{ Function, P.first->getTerminator() }] = P.second;
}

void ResultsPool::mergeCallSites(BasicBlock *Entry,
                                 const StackSizeMap &ToImport) {
  for (auto &P : ToImport) {
    FunctionCalls[Entry].push_back(P.first);
    CallSite Call = { Entry, P.first.callInstruction() };
    auto It = CallSites.find(Call);
    if (It != CallSites.end()) {
      if (!compareOptional(It->second, P.second)) {
        revng_abort("This call site has a stack at a different height than"
                    " previously recorded");
      }
    } else {
      CallSites[Call] = P.second;
    }
  }
}

/// \brief Helper class for computing the set of registers clobbered by each
///        function
///
/// This class basically takes each function, computes the set of all the
/// registers written by it and all of the functions in the transitive closure
/// of the callee set and remove the registers that are explicitly callee saved
/// (ECS).
///
/// This process is repeated multiple times to handle with increasing precision
/// recursive and indirect function calls, which are initially ignored. After
/// the first iteration, the recursive function calls use the result from the
/// previous iteration. On the other hand, indirect function calls are
/// considered clobbering all the registers except those that are ECS in the
/// majority of the functions.
struct ClobberedRegistersAnalysis {
  using ClobberedMap = std::map<llvm::BasicBlock *, std::set<int32_t>>;

  /// \brief Struct representing the result of a single iteration
  struct IterationResult {
  public:
    /// \brief Description of a register
    struct ECSVote {
    public:
      unsigned ECS; ///< Number of functions in which this register is ECS
      unsigned Total; ///< Number of functions writing this register

    public:
      bool operator==(const ECSVote &Other) const {
        return std::tie(ECS, Total) == std::tie(Other.ECS, Other.Total);
      }

      bool isECS() const { return ECS > Total / 2; }
    };

  public:
    ClobberedMap Clobbered;
    std::map<int32_t, ECSVote> ECSVotes;

  public:
    bool operator==(const IterationResult &Other) const {
      using std::tie;
      return tie(Clobbered, ECSVotes) == tie(Other.Clobbered, Other.ECSVotes);
    }

    bool operator!=(const IterationResult &Other) const {
      return not(*this == Other);
    }
  };

  /// \brief Compute an iteration
  static IterationResult
  recompute(ResultsPool &This, const ClobberedMap &InitialState) {
    // The results pool
    IterationResult Result;
    auto &Clobbered = Result.Clobbered;
    auto &ECSVotes = Result.ECSVotes;

    // Loop over all the functions
    for (auto &P : This.FunctionTypes) {
      BasicBlock *Function = P.first;

      // Have we handled this already?
      if (Clobbered.count(Function) != 0)
        continue;

      using iterator = typename std::vector<FunctionCall>::const_iterator;

      struct State {
      public:
        BasicBlock *Function;
        iterator CallIt;
        iterator EndCallIt;

      public:
        State(BasicBlock *Function, iterator CallIt, iterator EndCallIt) :
          Function(Function), CallIt(CallIt), EndCallIt(EndCallIt) {}
      };

      // Worklist
      std::deque<State> WorkList;

      // Set of currently in-progress functions, used to detect recursive
      // function calls
      std::set<BasicBlock *> InProgress;

      // Initialize the worklist
      const auto &FunctionCallsList = This.FunctionCalls[Function];
      WorkList.emplace_back(Function,
                            FunctionCallsList.begin(),
                            FunctionCallsList.end());
      InProgress.insert(Function);

      // Loop over the worklist
      while (not WorkList.empty()) {
        // Peek but don't pop
        State &Current = WorkList.back();
        BasicBlock *Function = Current.Function;

        // Get a reference to the results for the current function
        std::set<int32_t> &CurrentClobbered = Clobbered[Function];

        // Loop over the unprocessed function calls
        while (Current.CallIt != Current.EndCallIt) {
          // Get the callee
          BasicBlock *Callee = Current.CallIt->callee();

          if ((Callee == nullptr) or (InProgress.count(Callee) != 0)) {
            // Indirect or recursive function call, use result from last
            // iteration
            auto It = InitialState.find(Callee);
            if (It != InitialState.end())
              CurrentClobbered.insert(It->second.begin(), It->second.end());

          } else {
            // Do we already handle this callee?
            auto ClobberedIt = Clobbered.find(Callee);
            if (ClobberedIt == Clobbered.end()) {
              // No, push it on the worklist
              const auto &FunctionCallsList = This.FunctionCalls[Callee];
              WorkList.emplace_back(Callee,
                                    FunctionCallsList.begin(),
                                    FunctionCallsList.end());
              InProgress.insert(Callee);

              // Early exit so we can proceed from the callee
              break;
            }

            // OK, we already processed this callee

            // Merge in the clobbered set all those clobbered by the callee
            CurrentClobbered.insert(ClobberedIt->second.begin(),
                                    ClobberedIt->second.begin());
          }

          // Proceed to the next call site
          Current.CallIt++;
        }

        // Are we done?
        if (Current.CallIt == Current.EndCallIt) {
          // Oh, we're done

          {
            // Add all the locally written registers
            auto It = This.LocallyWrittenRegisters.find(Function);
            if (It != This.LocallyWrittenRegisters.end())
              CurrentClobbered.insert(It->second.begin(), It->second.end());
          }

          // Increase the counter associated to each written register
          for (int32_t Index : CurrentClobbered)
            ECSVotes[Index].Total++;

          {
            // Erase from the clobbered registers all the callee-saved, if any
            auto It = This.ExplicitlyCalleeSavedRegisters.find(Function);
            if (It != This.ExplicitlyCalleeSavedRegisters.end()) {
              // Do not use CurrentClobbered.erase(BeginIt, EndIt);
              for (int32_t Index : It->second)
                CurrentClobbered.erase(Index);

              // Increase the counter associated to each register
              for (int32_t Index : It->second)
                ECSVotes[Index].ECS++;
            }
          }

          // Pop from the worklist
          WorkList.pop_back();
          InProgress.erase(Function);
        }
      }
    }

    return Result;
  }

  /// \brief Repeat the analysis until a fixed point is reached
  static ClobberedMap run(ResultsPool &This) {
    IterationResult LastResult;
    IterationResult NewResult;

    do {
      LastResult = std::move(NewResult);

      // Use as initial state the previous iteration's state
      NewResult = recompute(This, LastResult.Clobbered);

      // Perform a majority vote on the result to associate to indirect function
      // calls
      std::set<int32_t> IndirectCallClobbered;
      for (auto &P : NewResult.ECSVotes) {

        // When a register is written, is it usually an ECS?
        if (not P.second.isECS()) {
          // No, consider it clobbered by indirect function calls
          IndirectCallClobbered.insert(P.first);
        }
      }

      // Assert the new set of registers clobbered by indirect function calls
      // contains at least all of the registers clobbered in the previous
      // iteration. If this is not the case, the algorithm might not converge.
      for (int32_t Clobbered : LastResult.Clobbered[nullptr])
        revng_assert(IndirectCallClobbered.count(Clobbered) != 0);

      // Save the results of the vote as the result associated with nullptr
      NewResult.Clobbered[nullptr] = std::move(IndirectCallClobbered);

    } while (LastResult != NewResult);

    return std::move(NewResult.Clobbered);
  }
};

FunctionsSummary ResultsPool::finalize(Module *M, Cache *TheCache) {
  ASID CPU = ASID::cpuID();

  // Create the result data structure
  FunctionsSummary Result;

  // Set function types
  for (auto &P : FunctionTypes)
    Result.Functions[P.first].Type = P.second;

  // Compute the set of registers clobbered by each function
  ClobberedRegistersAnalysis::ClobberedMap Clobbered;
  Clobbered = ClobberedRegistersAnalysis::run(*this);
  for (auto &P : Clobbered) {
    auto &Function = Result.Functions[P.first];
    for (int32_t Offset : P.second)
      Function.ClobberedRegisters.insert(TheCache->getCSVByIndex(Offset));
  }

  // Register block types
  for (auto &P : BranchesType) {
    BasicBlock *BB = P.first.branch()->getParent();
    Result.Functions[P.first.entry()].BasicBlocks[BB] = P.second;
  }

  using CallSiteDescription = FunctionsSummary::CallSiteDescription;

  //
  // Collect, for each call site, all the slots and create a CallSiteDescription
  //
  struct FunctionCallSites {
    /// \brief Collect all the slots used by the function/its callers
    std::set<ASSlot> Slots;
    /// \brief The callers
    std::map<CallSite, CallSiteDescription *> CallSites;
  };
  std::map<BasicBlock *, FunctionCallSites> FunctionCallSitesMap;

  auto &FCRA = FunctionCallRegisterArguments;
  auto &FCRV = FunctionCallReturnValues;
  auto &FRA = FunctionRegisterArguments;
  auto &FRV = FunctionReturnValues;

  for (auto &P : FRA) {
    BasicBlock *FunctionEntry = P.first.first;
    FunctionCallSites &FCS = FunctionCallSitesMap[FunctionEntry];
    auto Slot = ASSlot::create(CPU, P.first.second);
    FCS.Slots.insert(Slot);
  }

  for (auto &P : FRV) {
    BasicBlock *FunctionEntry = P.first.first;
    FunctionCallSites &FCS = FunctionCallSitesMap[FunctionEntry];
    auto Slot = ASSlot::create(CPU, P.first.second);
    FCS.Slots.insert(Slot);
  }

  // Go over arguments of function calls
  for (auto &P : FCRA) {
    const CallSite &TheCallSite = P.first.first;
    auto Slot = ASSlot::create(CPU, P.first.second);
    BasicBlock *CallerBB = TheCallSite.callInstruction()->getParent();
    BasicBlock *Callee = getFunctionCallCallee(CallerBB);
    FunctionCallSites &FCS = FunctionCallSitesMap[Callee];

    // Register the slot
    FCS.Slots.insert(Slot);

    // Check if we already created the CallSiteDescription
    auto It = FCS.CallSites.find(TheCallSite);
    if (It == FCS.CallSites.end()) {
      auto &CallerCallSites = Result.Functions[TheCallSite.caller()].CallSites;
      Instruction *I = TheCallSite.callInstruction();
      CallerCallSites.emplace_back(I, Callee);
      FCS.CallSites[TheCallSite] = &CallerCallSites.back();
      revng_assert(FCS.CallSites[TheCallSite] == &CallerCallSites.back());
    }
  }

  // Go over return values of function calls
  for (auto &P : FCRV) {
    const CallSite &TheCallSite = P.first.first;
    auto Slot = ASSlot::create(CPU, P.first.second);
    BasicBlock *CallerBB = TheCallSite.callInstruction()->getParent();
    BasicBlock *Callee = getFunctionCallCallee(CallerBB);
    FunctionCallSites &FCS = FunctionCallSitesMap[Callee];
    FCS.Slots.insert(Slot);
  }

  //
  // Merge information about a function and all the call sites targeting it
  //

  // For each function
  for (auto &P : Result.Functions) {
    BasicBlock *FunctionEntry = P.first;

    // Integrate slots from each call site
    FunctionCallSites &FCS = FunctionCallSitesMap[FunctionEntry];

    // Iterate over each slot
    for (ASSlot Slot : FCS.Slots) {
      revng_assert(Slot.addressSpace() == CPU);
      int32_t Offset = Slot.offset();
      if (not TheCache->isCSVIndex(Offset))
        continue;

      GlobalVariable *CSV = TheCache->getCSVByIndex(Offset);
      FunctionSlot TheFunctionSlot{ FunctionEntry, Offset };

      bool CalleeHasSlot = FRA.count(TheFunctionSlot) != 0;
      if (FunctionEntry == nullptr or not CalleeHasSlot) {
        for (auto &Q : FCS.CallSites) {
          CallSiteDescription &TheCallSiteDescription = *Q.second;
          auto &CallSiteRegister = TheCallSiteDescription.RegisterSlots[CSV];
          const CallSite &TheCallSite = Q.first;
          FunctionCallSlot FCS{ TheCallSite, Offset };

          CallSiteRegister.Argument = FCRA[FCS];
          CallSiteRegister.Argument.notAvailable();
          CallSiteRegister.ReturnValue = FCRV[FCS];
          CallSiteRegister.ReturnValue.notAvailable();

          if (FunctionEntry != nullptr) {
            using FRegisterArgument = FunctionRegisterArgument;
            using FReturnValue = FunctionReturnValue;
            auto &Slot = P.second.RegisterSlots[CSV];
            Slot.Argument = FRegisterArgument(FRegisterArgument::Maybe);
            Slot.ReturnValue = FReturnValue(FReturnValue::Maybe);
          }
        }

        continue;
      }

      {
        //
        // Merge arguments
        //

        // Register status at the function
        FunctionRegisterArgument FunctionStatus = FRA[TheFunctionSlot];
        auto Status = FunctionStatus.value();
        revng_assert(Status == FunctionRegisterArgument::Maybe
                     or Status == FunctionRegisterArgument::NoOrDead
                     or Status == FunctionRegisterArgument::Contradiction
                     or Status == FunctionRegisterArgument::Yes
                     or Status == FunctionRegisterArgument::No);

        // Check if at least a call site says yes
        bool AtLeastAYes = false;
        for (auto &Q : FCS.CallSites) {
          const CallSite &TheCallSite = Q.first;
          revng_assert(Q.second != nullptr);
          CallSiteDescription &TheCallSiteDescription = *Q.second;
          FunctionCallSlot FCS{ TheCallSite, Offset };

          const FunctionCallRegisterArgument &CallerStatus = FCRA[FCS];
          auto Status = CallerStatus.value();
          revng_assert(Status == FunctionCallRegisterArgument::Maybe
                       or Status == FunctionCallRegisterArgument::Yes);

          // Register if there's at least a Yes
          if (Status == FunctionCallRegisterArgument::Yes) {
            AtLeastAYes = true;
            break;
          }

        }

        if (AtLeastAYes) {
          // Propagate the yes to the function
          switch (FunctionStatus.value()) {
          case FunctionRegisterArgument::Maybe:
            FunctionStatus = FunctionRegisterArgument(FunctionRegisterArgument::Yes);
            break;
          case FunctionRegisterArgument::NoOrDead:
            FunctionStatus = FunctionRegisterArgument(FunctionRegisterArgument::Dead);
            break;
          case FunctionRegisterArgument::Contradiction:
          case FunctionRegisterArgument::Yes:
          case FunctionRegisterArgument::No:
            // Do nothing
            break;
          default:
            revng_abort();
          }
        }

        // Register the result for the argument of the function
        P.second.RegisterSlots[CSV].Argument = FunctionStatus;

        // Go over the call sites again and propagate from the result in the
        // function to the call sites
        for (auto &Q : FCS.CallSites) {
          const CallSite &TheCallSite = Q.first;
          revng_assert(Q.second != nullptr);
          CallSiteDescription &TheCallSiteDescription = *Q.second;
          FunctionCallSlot FCS{ TheCallSite, Offset };

          const FunctionCallRegisterArgument &CallerStatus = FCRA[FCS];

          // Update the status at the call site, starting from the status of the
          // callee
          FunctionCallRegisterArgument Result;
          using FCRegisterArgument = FunctionCallRegisterArgument;
          switch (FunctionStatus.value()) {
          case FunctionRegisterArgument::Maybe:
            Result = FCRegisterArgument(FCRegisterArgument::Maybe);
            break;
          case FunctionRegisterArgument::NoOrDead:
            Result = FCRegisterArgument(FCRegisterArgument::NoOrDead);
            break;
          case FunctionRegisterArgument::Dead:
            Result = FCRegisterArgument(FCRegisterArgument::Dead);
            break;
          case FunctionRegisterArgument::Contradiction:
            Result = FCRegisterArgument(FCRegisterArgument::Contradiction);
            break;
          case FunctionRegisterArgument::Yes:
            Result = FCRegisterArgument(FCRegisterArgument::Yes);
            break;
          case FunctionRegisterArgument::No:
            Result = FCRegisterArgument(FCRegisterArgument::No);
            break;
          default:
            revng_abort();
          }

          // If the callee doesn't say No and the caller says yes
          if (FunctionStatus.value() != FunctionRegisterArgument::No
              and CallerStatus.value() == FCRegisterArgument::Yes) {
            // Promote caller using the Yes information
            switch (FunctionStatus.value()) {
            case FunctionRegisterArgument::NoOrDead:
              Result = FCRegisterArgument(FCRegisterArgument::Dead);
              break;
            case FunctionRegisterArgument::Maybe:
              Result = FCRegisterArgument(FCRegisterArgument::Yes);
              break;
            case FunctionRegisterArgument::Dead:
            case FunctionRegisterArgument::Contradiction:
            case FunctionRegisterArgument::Yes:
              // Do nothing
              break;
            default:
              revng_abort();
            }
          }

          // Register the result
          TheCallSiteDescription.RegisterSlots[CSV].Argument = Result;
        }

      }

      {
        //
        // Merge return values
        //

        // Register status at the function
        const FunctionReturnValue &FunctionStatus = FRV[TheFunctionSlot];
        auto Status = FunctionStatus.value();
        revng_assert(Status == FunctionReturnValue::Maybe
                     or Status == FunctionReturnValue::No
                     or Status == FunctionReturnValue::YesOrDead);

        // Propagate information from the function to callers (and record if at
        // least on call sites says Yes or Dead)
        bool AtLeastAYesOrDead = false;
        for (auto &Q : FCS.CallSites) {
          const CallSite &TheCallSite = Q.first;
          auto &TheCallSiteDescription = *Q.second;
          FunctionCallSlot FCS{ TheCallSite, Offset };

          // Register status at current call site
          const FunctionCallReturnValue &CallerStatus = FCRV[FCS];
          auto Status = CallerStatus.value();
          revng_assert(Status == FunctionCallReturnValue::Maybe
                       or Status == FunctionCallReturnValue::NoOrDead
                       or Status == FunctionCallReturnValue::Yes
                       or Status == FunctionCallReturnValue::Contradiction);

          FunctionCallReturnValue Result = CallerStatus;

          switch (FunctionStatus.value()) {
          case FunctionReturnValue::No:
            // No from the function is propagated as is
            Result = FunctionCallReturnValue::no();
            break;
          case FunctionReturnValue::YesOrDead:
            // Propagate the strong yes information
            switch (CallerStatus.value()) {
            case FunctionCallReturnValue::Maybe:
              Result = FunctionCallReturnValue(FunctionCallReturnValue::Yes);
              break;
            case FunctionCallReturnValue::NoOrDead:
              Result = FunctionCallReturnValue(FunctionCallReturnValue::Dead);
              break;
            case FunctionCallReturnValue::Yes:
            case FunctionCallReturnValue::Contradiction:
              // Do nothing
              break;
            default:
              revng_abort();
            }
            break;
          case FunctionReturnValue::Maybe:
            break;
          default:
            revng_abort();
          }

          // In all other cases, no changes

          // Record if at least one result is Yes or Dead
          {
            auto Status = Result.value();
            AtLeastAYesOrDead = (AtLeastAYesOrDead
                                 or Status == FunctionCallReturnValue::Yes
                                 or Status == FunctionCallReturnValue::Dead);
          }

          // Register the result for this call site
          TheCallSiteDescription.RegisterSlots[CSV].ReturnValue = Result;
        }

        // Cross-contamination of callers
        if (AtLeastAYesOrDead) {
          // If at least a call site states that this slot is a return value,
          // all the other call sites can benefit from this information

          for (auto &Q : FCS.CallSites) {
            using FCReturnValue = FunctionCallReturnValue;
            auto &TheCallSiteDescription = *Q.second;
            auto &Value = TheCallSiteDescription.RegisterSlots[CSV].ReturnValue;
            switch (Value.value()) {
            case FCReturnValue::NoOrDead:
              Value = FCReturnValue(FCReturnValue::Dead);
              break;
            case FCReturnValue::Maybe:
              Value = FCReturnValue(FCReturnValue::YesOrDead);
              break;
            case FCReturnValue::Yes:
            case FCReturnValue::Dead:
            case FCReturnValue::Contradiction:
              // Do nothing
              break;
            case FCReturnValue::No:
            default:
              revng_abort();
            }
          }
        }

        // Update the result associated to the function
        FunctionReturnValue Result = FunctionStatus;
        using FCReturnValue = FunctionCallReturnValue;

        if (FCS.CallSites.size() > 0) {
          // At this point the information associated to the call sites is
          // either all "No", one of "Yes", "Dead" and "YesOrDead" or one of
          // "NoOrDead" and "Maybe"
          bool AllNo = true;
          bool AllYesOrDead = true;
          bool AllNoOrDead = true;

          // Initialize the result to propagate to the callee with the first
          // call site
          auto BeginIt = FCS.CallSites.begin();
          auto Accumulate = BeginIt->second->RegisterSlots[CSV].ReturnValue;

          for (auto &Q : FCS.CallSites) {
            auto &TheCallSiteDescription = *Q.second;
            auto &Value = TheCallSiteDescription.RegisterSlots[CSV].ReturnValue;

            AllNo = AllNo and Value.value() == FCReturnValue::No;

            auto Status = Value.value();
            AllYesOrDead = (AllYesOrDead
                            and (Status == FCReturnValue::Yes
                                 or Status == FCReturnValue::Dead
                                 or Status == FCReturnValue::YesOrDead));

            AllNoOrDead = (AllNoOrDead
                           and (Status == FCReturnValue::NoOrDead
                                or Status == FCReturnValue::Maybe));

            // If the value has changed, move towards the most generic
            if (Status != Accumulate.value()) {
              using FCReturnValue = FCReturnValue;
              switch (Status) {
              case FCReturnValue::Yes:
              case FCReturnValue::Dead:
              case FCReturnValue::YesOrDead:
                Accumulate = FCReturnValue(FCReturnValue::YesOrDead);
                break;
              case FCReturnValue::NoOrDead:
              case FCReturnValue::Maybe:
                Accumulate = FCReturnValue(FCReturnValue::Maybe);
                break;
              case FCReturnValue::No:
              case FCReturnValue::Contradiction:
                revng_abort();
              }
            }
          }

          // AllNo XOR AllYesOrDead XOR AllNoOrDead
          revng_assert((AllNo and not(AllYesOrDead or AllNoOrDead))
                       or (AllYesOrDead and not(AllNo or AllNoOrDead))
                       or (AllNoOrDead and not(AllYesOrDead or AllNo)));

          // Propagate the information from callers to function

          // If AllNo, nothing to do
          using FReturnValue = FunctionReturnValue;
          bool IsNo = FunctionStatus.value() == FReturnValue::No;
          revng_assert(AllNo ? IsNo : true);

          // If the function status was maybe, we might have something to
          // promote in the call
          if (FunctionStatus.value() == FReturnValue::Maybe) {
            switch (Accumulate.value()) {
            case FCReturnValue::Yes:
            case FCReturnValue::Dead:
            case FCReturnValue::YesOrDead:
              Result = FReturnValue(FReturnValue::YesOrDead);
              break;
            case FCReturnValue::NoOrDead:
              Result = FReturnValue(FReturnValue::NoOrDead);
              break;
            case FCReturnValue::Maybe:
              Result = FReturnValue(FReturnValue::Maybe);
              break;
            default:
              revng_abort();
            }
          }
        }

        // Register the result with the function
        P.second.RegisterSlots[CSV].ReturnValue = Result;
      }
    }
  }

  return Result;
}

} // namespace StackAnalysis
