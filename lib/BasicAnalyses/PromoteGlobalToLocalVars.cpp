/// \file PromoteGlobalToLocalVars.cpp
/// \brief Promote CSVs in form of global variables to local variables.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/IRBuilder.h"

#include "revng/BasicAnalyses/PromoteGlobalToLocalVars.h"
#include "revng/Support/IRHelpers.h"

llvm::PreservedAnalyses
PromoteGlobalToLocalPass::run(llvm::Function &F,
                              llvm::FunctionAnalysisManager &FAM) {
  using namespace llvm;

  // Collect the CSVs used by the current function.
  std::map<GlobalVariable *, Value *> CSVMap;
  for (auto &BB : F) {
    for (auto &I : BB) {
      Value *Pointer = nullptr;
      if (auto *Load = dyn_cast<LoadInst>(&I))
        Pointer = skipCasts(Load->getPointerOperand());
      else if (auto *Store = dyn_cast<StoreInst>(&I))
        Pointer = skipCasts(Store->getPointerOperand());
      else
        continue;

      if (auto *CSV = dyn_cast_or_null<GlobalVariable>(Pointer))
        CSVMap.try_emplace(CSV);
    }
  }

  // Create an equivalent local variable, replace all the uses of the CSV.
  IRBuilder<> Builder(&F.getEntryBlock().front());
  for (const auto &[CSV, _] : CSVMap) {
    auto *CSVTy = CSV->getType()->getPointerElementType();
    auto *Alloca = Builder.CreateAlloca(CSVTy, nullptr, CSV->getName());
    replaceAllUsesInFunctionWith(&F, CSV, Alloca);

    CSVMap[CSV] = Alloca;
  }

  // Load all the CSVs and store their value onto the local variables.
  for (const auto &[CSV, Alloca] : CSVMap)
    Builder.CreateStore(Builder.CreateLoad(CSV), Alloca);

  return PreservedAnalyses::none();
}
