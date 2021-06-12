#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Support/revng.h"

class SegregateDirectStackAccessesPass
  : public llvm::PassInfoMixin<SegregateDirectStackAccessesPass> {
  llvm::LLVMContext *Context;
  GeneratedCodeBasicInfo *GCBI;
  std::vector<llvm::Instruction *> DirectStackAccesses;
  std::vector<llvm::Instruction *> NotDirectStackAccesses;

public:
  SegregateDirectStackAccessesPass() = default;

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);

private:
  void segregateAccesses(llvm::Function &F);
  void decorateStackAccesses();
};
