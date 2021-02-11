/// \file RemoveNewPCCalls.cpp
/// \brief Remove calls to newpc in a function.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/SmallVector.h"

#include "revng/BasicAnalyses/RemoveNewPCCalls.h"
#include "revng/Support/IRHelpers.h"

llvm::PreservedAnalyses
RemoveNewPCCallsPass::run(llvm::Function &F,
                          llvm::FunctionAnalysisManager &FAM) {
  llvm::SmallVector<llvm::Instruction *, 16> ToErase;
  for (auto &BB : F) {
    for (auto &I : BB)
      if (auto *C = dyn_cast<llvm::CallInst>(&I))
        if (getCallee(C)->getName() == "newpc")
          ToErase.push_back(C);
  }

  bool Changed = not ToErase.empty();
  for (auto *I : ToErase)
    I->eraseFromParent();

  return (Changed ? llvm::PreservedAnalyses::none() :
                    llvm::PreservedAnalyses::all());
}
