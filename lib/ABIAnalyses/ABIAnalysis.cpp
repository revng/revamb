//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/ABIAnalyses/ABIAnalysis.h"
#include "revng/ABIAnalyses/Common.h"
#include "revng/ADT/ZipMapIterator.h"
#include "revng/Model/Binary.h"
#include "revng/Support/Assert.h"
#include "revng/Support/Debug.h"
#include "revng/Support/MetaAddress.h"

#include "Analyses.h"

using namespace llvm;

static Logger<> ABIAnalysesLog("abi-analyses");

namespace ABIAnalyses {
using RegisterState = model::RegisterState::Values;

struct PartialAnalysisResults {
  // Per function analysis
  RegisterStateMap UAOF;
  RegisterStateMap DRAOF;

  // Per call site analysis
  std::map<std::pair<MetaAddress, BasicBlock *>, RegisterStateMap> URVOFC;
  std::map<std::pair<MetaAddress, BasicBlock *>, RegisterStateMap> RAOFC;
  std::map<std::pair<MetaAddress, BasicBlock *>, RegisterStateMap> DRVOFC;

  // Per return analysis
  std::map<std::pair<MetaAddress, BasicBlock *>, RegisterStateMap> URVOF;

  // Debug methods
  void dump() const debug_function { dump(dbg, ""); }

  template<typename T>
  void dump(T &Output, const char *Prefix) const;
};

// Print the analysis results
template<typename T>
void PartialAnalysisResults::dump(T &Output, const char *Prefix) const {
  Output << Prefix << "UsedArgumentsOfFunction:\n";
  for (auto &[GV, State] : UAOF) {
    Output << Prefix << "  " << GV->getName().str() << " = "
           << model::RegisterState::getName(State).str() << '\n';
  }

  Output << Prefix << "DeadRegisterArgumentsOfFunction:\n";
  for (auto &[GV, State] : DRAOF) {
    Output << Prefix << "  " << GV->getName().str() << " = "
           << model::RegisterState::getName(State).str() << '\n';
  }

  Output << Prefix << "UsedReturnValuesOfFunctionCall:\n";
  for (auto &[Key, StateMap] : URVOFC) {
    Output << Prefix << "  " << Key.second->getName().str() << '\n';
    for (auto &[GV, State] : StateMap) {
      Output << Prefix << "    " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
  }

  Output << Prefix << "RegisterArgumentsOfFunctionCall:\n";
  for (auto &[Key, StateMap] : RAOFC) {
    Output << Prefix << "  " << Key.second->getName().str() << '\n';
    for (auto &[GV, State] : StateMap) {
      Output << Prefix << "    " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
  }

  Output << Prefix << "DeadReturnValuesOfFunctionCall:\n";
  for (auto &[Key, StateMap] : DRVOFC) {
    Output << Prefix << "  " << Key.second->getName().str() << '\n';
    for (auto &[GV, State] : StateMap) {
      Output << Prefix << "    " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
  }

  Output << Prefix << "UsedReturnValuesOfFunction:\n";
  for (auto &[Key, StateMap] : URVOF) {
    Output << Prefix << "  " << Key.second->getName().str() << '\n';
    for (auto &[GV, State] : StateMap) {
      Output << Prefix << "  " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
  }
}

static RegisterState combine(RegisterState LH, RegisterState RH) {
  switch (LH) {
  case RegisterState::Yes:
    switch (RH) {
    case RegisterState::Yes:
    case RegisterState::YesOrDead:
    case RegisterState::Maybe:
      return RegisterState::Yes;
    case RegisterState::No:
    case RegisterState::NoOrDead:
    case RegisterState::Dead:
    case RegisterState::Contradiction:
      return RegisterState::Contradiction;
    case RegisterState::Invalid:
      revng_abort();
    }
    break;

  case RegisterState::YesOrDead:
    switch (RH) {
    case RegisterState::Yes:
      return RegisterState::Yes;
    case RegisterState::Maybe:
    case RegisterState::YesOrDead:
      return RegisterState::YesOrDead;
    case RegisterState::Dead:
    case RegisterState::NoOrDead:
      return RegisterState::Dead;
    case RegisterState::No:
    case RegisterState::Contradiction:
      return RegisterState::Contradiction;
    case RegisterState::Invalid:
      revng_abort();
    }
    break;

  case RegisterState::No:
    switch (RH) {
    case RegisterState::No:
    case RegisterState::NoOrDead:
    case RegisterState::Maybe:
      return RegisterState::No;
    case RegisterState::Yes:
    case RegisterState::YesOrDead:
    case RegisterState::Dead:
    case RegisterState::Contradiction:
      return RegisterState::Contradiction;
    case RegisterState::Invalid:
      revng_abort();
    }
    break;

  case RegisterState::NoOrDead:
    switch (RH) {
    case RegisterState::No:
      return RegisterState::No;
    case RegisterState::Maybe:
    case RegisterState::NoOrDead:
      return RegisterState::NoOrDead;
    case RegisterState::Dead:
    case RegisterState::YesOrDead:
      return RegisterState::Dead;
    case RegisterState::Yes:
    case RegisterState::Contradiction:
      return RegisterState::Contradiction;
    case RegisterState::Invalid:
      revng_abort();
    }
    break;

  case RegisterState::Dead:
    switch (RH) {
    case RegisterState::Dead:
    case RegisterState::Maybe:
    case RegisterState::NoOrDead:
    case RegisterState::YesOrDead:
      return RegisterState::Dead;
    case RegisterState::No:
    case RegisterState::Yes:
    case RegisterState::Contradiction:
      return RegisterState::Contradiction;
    case RegisterState::Invalid:
      revng_abort();
    }
    break;

  case RegisterState::Maybe:
    return RH;

  case RegisterState::Contradiction:
    return RegisterState::Contradiction;

  case RegisterState::Invalid:
    revng_abort();

  default:
    revng_abort();
  }
}

void finalizeReturnValues(ABIAnalysesResults &ABIResults) {
  for (auto &[PC, RSMap] : ABIResults.ReturnValues) {
    for (auto &[CSV, RS] : RSMap) {
      if (ABIResults.FinalReturnValues.count(CSV) == 0)
        ABIResults.FinalReturnValues[CSV] = RegisterState::Maybe;

      ABIResults
        .FinalReturnValues[CSV] = combine(ABIResults.FinalReturnValues[CSV],
                                          RS);
    }
  }
}

// Run the ABI analyses on the outlined function F. This function must have all
// the original function calls replaced with a basic block starting with a call
// to @precall_hook followed by a summary of the side effects of the function
// followed by a call to @postcall_hook and a basic block terminating
// instruction.
ABIAnalysesResults analyzeOutlinedFunction(Function *F,
                                           const GeneratedCodeBasicInfo &GCBI,
                                           Function *CallSiteHook,
                                           Function *RetHook) {
  ABIAnalysesResults FinalResults;
  PartialAnalysisResults Results;

  // Initial population of partial results.
  Results.UAOF = UsedArgumentsOfFunction::analyze(&F->getEntryBlock(), GCBI);
  Results.DRAOF = DeadRegisterArgumentsOfFunction::analyze(&F->getEntryBlock(),
                                                           GCBI);
  for (auto &I : instructions(F)) {
    BasicBlock *BB = I.getParent();

    if (auto *Call = dyn_cast<CallInst>(&I)) {
      MetaAddress PC;
      if (Call->getCalledFunction() == CallSiteHook
          || Call->getCalledFunction() == RetHook)
        PC = MetaAddress::fromConstant(Call->getArgOperand(0));

      if (Call->getCalledFunction() == CallSiteHook) {
        // BB is a call site.
        Results
          .URVOFC[{ PC, BB }] = UsedReturnValuesOfFunctionCall::analyze(BB,
                                                                        GCBI);
        Results
          .RAOFC[{ PC, BB }] = RegisterArgumentsOfFunctionCall::analyze(BB,
                                                                        GCBI);
        Results
          .DRVOFC[{ PC, BB }] = DeadReturnValuesOfFunctionCall::analyze(BB,
                                                                        GCBI);
      } else if (Call->getCalledFunction() == RetHook) {
        Results.URVOF[{ PC, BB }] = UsedReturnValuesOfFunction::analyze(BB,
                                                                        GCBI);
      }
    }
  }

  if (ABIAnalysesLog.isEnabled()) {
    ABIAnalysesLog << "Dumping ABIAnalyses results for function "
                   << F->getName() << ": \n";
    Results.dump();
  }

  // Finalize results. Combine UAOF and DRAOF.
  for (auto &[Left, Right] : zipmap_range(Results.UAOF, Results.DRAOF)) {
    auto *CSV = Left == nullptr ? Right->first : Left->first;
    RegisterState LV = Left == nullptr ? RegisterState::Maybe : Left->second;
    RegisterState RV = Right == nullptr ? RegisterState::Maybe : Right->second;
    FinalResults.Arguments[CSV] = combine(LV, RV);
  }

  // Add RAOFC.
  for (auto &[Key, RSMap] : Results.RAOFC) {
    auto PC = Key.first;
    for (auto &[CSV, RS] : RSMap)
      FinalResults.CallSites[PC].Arguments[CSV] = RS;
  }

  // Combine URVOFC and DRVOFC.
  for (auto &[Key, _] : Results.URVOFC) {
    auto PC = Key.first;
    for (auto &[Left, Right] :
         zipmap_range(Results.URVOFC[Key], Results.DRVOFC[Key])) {
      auto *CSV = Left == nullptr ? Right->first : Left->first;
      RegisterState LV = Left == nullptr ? RegisterState::Maybe : Left->second;
      RegisterState RV = Right == nullptr ? RegisterState::Maybe :
                                            Right->second;
      FinalResults.CallSites[PC].ReturnValues[CSV] = combine(LV, RV);
    }
  }

  // Add URVOF.
  for (auto &[Key, RSMap] : Results.URVOF) {
    auto PC = Key.first;
    for (auto &[CSV, RS] : RSMap)
      FinalResults.ReturnValues[PC][CSV] = RS;
  }

  return FinalResults;
}

template<typename T>
void ABIAnalysesResults::dump(T &Output, const char *Prefix) const {
  Output << Prefix << "Arguments:\n";
  for (auto &[GV, State] : Arguments) {
    Output << Prefix << "  " << GV->getName().str() << " = "
           << model::RegisterState::getName(State).str() << '\n';
  }

  Output << Prefix << "Call site:\n";
  for (auto &[PC, StateMap] : CallSites) {
    Output << Prefix << "  " << PC.address() << '\n';
    Output << Prefix << "  "
           << "  "
           << "Arguments:\n";
    for (auto &[GV, State] : StateMap.Arguments) {
      Output << Prefix << "    " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
    Output << Prefix << "  "
           << "  "
           << "Return values:\n";
    for (auto &[GV, State] : StateMap.ReturnValues) {
      Output << Prefix << "    " << GV->getName().str() << " = "
             << model::RegisterState::getName(State).str() << '\n';
    }
  }

  Output << Prefix << "Return values:\n";
  for (auto &[GV, State] : FinalReturnValues) {
    Output << Prefix << "  " << GV->getName().str() << " = "
           << model::RegisterState::getName(State).str() << '\n';
  }
}

} // namespace ABIAnalyses
