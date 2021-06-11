#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/AssemblyAnnotationWriter.h"
#include "llvm/IR/PassManager.h"

#include "revng/Support/revng.h"

class AAWriterPass : public llvm::PassInfoMixin<AAWriterPass> {
  const bool StoresOnly;

public:
  AAWriterPass(bool StoresOnly = false) : StoresOnly(StoresOnly) {}

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);
};
