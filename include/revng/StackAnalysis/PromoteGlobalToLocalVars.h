#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/PassManager.h"

class PromoteGlobalToLocalPass
  : public llvm::PassInfoMixin<PromoteGlobalToLocalPass> {

public:
  PromoteGlobalToLocalPass() {}

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);
};
