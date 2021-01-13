#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Support/revng.h"

class CSVAliasAnalysisPass : public llvm::PassInfoMixin<CSVAliasAnalysisPass> {
  struct CSVAliasInfo {
    llvm::MDNode *AliasScope;
    llvm::MDNode *AliasSet;
    llvm::MDNode *NoAliasSet;
  };
  std::map<const llvm::GlobalVariable *, CSVAliasInfo> CSVAliasInfoMap;
  std::vector<llvm::Metadata *> AllCSVScopes;
  GeneratedCodeBasicInfo *GCBI;

public:
  CSVAliasAnalysisPass() = default;

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);

private:
  void initializeAliasInfo(llvm::Module &M);
  void decorateMemoryAccesses(llvm::Instruction &I);
};
