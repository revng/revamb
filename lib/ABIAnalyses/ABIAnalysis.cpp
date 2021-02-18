//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/ABIAnalyses/ABIAnalysis.h"
#include "revng/ABIAnalyses/Common.h"
#include "revng/Model/Binary.h"
#include "revng/Support/Debug.h"

#include "Analyses.h"

using namespace llvm;

namespace ABIAnalyses {

// Run the ABI analyses on the outlined function F. This function must have all
// the original function calls replaced with a basic block starting with a call
// to @precall_hook followed by a summary of the side effects of the function
// followed by a call to @postcall_hook and a basic block terminating
// instruction.
AnalysisResults analyzeOutlinedFunction(Function *F,
                                        const GeneratedCodeBasicInfo &GCBI,
                                        Function *CallSiteHook) {
  AnalysisResults Results;

  Results.UAOF = UsedArgumentsOfFunction::analyze(&F->getEntryBlock(), GCBI);
  Results.DRAOF = DeadRegisterArgumentsOfFunction::analyze(&F->getEntryBlock(),
                                                           GCBI);
  for (auto &I : instructions(F)) {
    BasicBlock *BB = I.getParent();

    if (auto *C = dyn_cast<CallInst>(&I)) {
      if (C->getCalledFunction() == CallSiteHook) {
        // BB is a call site.
        Results.URVOFC[BB] = UsedReturnValuesOfFunctionCall::analyze(BB, GCBI);
        Results.RAOFC[BB] = RegisterArgumentsOfFunctionCall::analyze(BB, GCBI);
        Results.DRVOFC[BB] = DeadReturnValuesOfFunctionCall::analyze(BB, GCBI);
      }
    } else if (auto *R = dyn_cast<ReturnInst>(&I)) {
      Results.URVOF[BB] = UsedReturnValuesOfFunction::analyze(BB, GCBI);
    }
  }

  return Results;
}

} // namespace ABIAnalyses
