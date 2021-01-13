/// \file CSVAliasAnalysis.cpp
/// \brief Decorate memory accesses with information about CSV aliasing.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/PassManager.h"
#include "llvm/IR/Value.h"

#include "revng/BasicAnalyses/CSVAliasAnalysis.h"
#include "revng/Support/Assert.h"

using namespace llvm;
using CSVAAP = CSVAliasAnalysisPass;

PreservedAnalyses CSVAAP::run(Function &F, FunctionAnalysisManager &FAM) {
  // Get the result of the GCBI analysis
  auto &M = *F.getParent();
  const auto &MAMProxy = FAM.getResult<ModuleAnalysisManagerFunctionProxy>(F);
  GCBI = MAMProxy.getCachedResult<GeneratedCodeBasicInfoAnalysis>(M);
  if (!GCBI)
    GCBI = &(FAM.getResult<GeneratedCodeBasicInfoAnalysis>(F));
  revng_assert(GCBI != nullptr);

  // Initialize the alias information for the CSVs.
  initializeAliasInfo(M);

  // Decorate the IR with the alias information for the CSVs.
  for (BasicBlock &BB : F)
    for (Instruction &I : BB)
      decorateMemoryAccesses(I);

  return PreservedAnalyses::none();
}

void CSVAAP::initializeAliasInfo(Module &M) {
  LLVMContext &Context = M.getContext();
  QuickMetadata QMD(Context);
  MDBuilder MDB(Context);

  MDNode *AliasDomain = MDB.createAliasScopeDomain("CSVAliasDomain");
  std::vector<GlobalVariable *> CSVs = GCBI->csvs();

  const auto *PCH = GCBI->programCounterHandler();
  for (GlobalVariable *PCCSV : PCH->pcCSVs())
    CSVs.emplace_back(PCCSV);

  // Build alias scopes
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];

    std::string Name = CSV->getName().str();
    MDNode *CSVScope = MDB.createAliasScope(Name, AliasDomain);
    AliasInfo.AliasScope = CSVScope;
    AllCSVScopes.push_back(CSVScope);
    MDNode *CSVAliasSet = MDNode::get(Context,
                                      ArrayRef<Metadata *>({ CSVScope }));
    AliasInfo.AliasSet = CSVAliasSet;
  }

  // Build noalias sets
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];
    std::vector<Metadata *> OtherCSVScopes;
    for (const auto &Q : CSVAliasInfoMap)
      if (Q.first != CSV)
        OtherCSVScopes.push_back(Q.second.AliasScope);

    MDNode *CSVNoAliasSet = MDNode::get(Context, OtherCSVScopes);
    AliasInfo.NoAliasSet = CSVNoAliasSet;
  }
}

void CSVAAP::decorateMemoryAccesses(Instruction &I) {
  Value *Ptr = nullptr;

  if (auto *L = dyn_cast<LoadInst>(&I))
    Ptr = L->getPointerOperand();
  else if (auto *S = dyn_cast<StoreInst>(&I))
    Ptr = S->getPointerOperand();
  else
    return;

  // Check if the pointer is a CSV
  if (auto *GV = dyn_cast<GlobalVariable>(Ptr)) {
    auto It = CSVAliasInfoMap.find(GV);
    if (It != CSVAliasInfoMap.end()) {
      // Set alias.scope and noalias metadata
      I.setMetadata(LLVMContext::MD_alias_scope,
                    MDNode::concatenate(I.getMetadata(
                                          LLVMContext::MD_alias_scope),
                                        It->second.AliasSet));
      I.setMetadata(LLVMContext::MD_noalias,
                    MDNode::concatenate(I.getMetadata(LLVMContext::MD_noalias),
                                        It->second.NoAliasSet));
      return;
    }
  }

  // It's not a CSV memory access, set noalias info
  MDNode *MemoryAliasSet = MDNode::get(I.getContext(), AllCSVScopes);
  I.setMetadata(LLVMContext::MD_noalias,
                MDNode::concatenate(I.getMetadata(LLVMContext::MD_noalias),
                                    MemoryAliasSet));
}
