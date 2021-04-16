#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Value.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Support/revng.h"

struct CSVAliasAnalysisInterface {
  GeneratedCodeBasicInfo *GCBI;
  llvm::MDNode *AliasDomain;
  friend class SegregateDirectStackAccesses;

protected:
  CSVAliasAnalysisInterface() : GCBI(nullptr) {}
};

template<bool SegregateStackAccesses = false>
class CSVAliasAnalysisPass
  : public CSVAliasAnalysisInterface,
    public llvm::PassInfoMixin<CSVAliasAnalysisPass<SegregateStackAccesses>> {
  struct CSVAliasInfo {
    llvm::MDNode *AliasScope;
    llvm::MDNode *AliasSet;
    llvm::MDNode *NoAliasSet;
  };
  std::map<const llvm::GlobalVariable *, CSVAliasInfo> CSVAliasInfoMap;
  std::vector<llvm::Metadata *> AllCSVScopes;

public:
  CSVAliasAnalysisPass() = default;

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &FAM);

private:
  void initializeAliasInfo(llvm::Module &M);
  void decorateMemoryAccesses(llvm::Instruction &I);
};
