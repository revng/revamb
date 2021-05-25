#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/PassManager.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Support/revng.h"

class IndirectBranchInfoPrinterPass
  : public llvm::PassInfoMixin<IndirectBranchInfoPrinterPass> {
  GeneratedCodeBasicInfo *GCBI;

public:
  IndirectBranchInfoPrinterPass() : GCBI(nullptr) {}

  void serialize(llvm::raw_ostream &,
                 llvm::CallBase *Call,
                 llvm::SmallVectorImpl<llvm::GlobalVariable *> &);

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);
};
