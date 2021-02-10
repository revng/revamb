/// \file RemoveHelperCalls.cpp
/// \brief Remove calls to helpers in a function and replaces them
///        with stores of an opaque value onto the CSVs clobbered by the helper.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/SmallVector.h"

#include "revng/BasicAnalyses/RemoveHelperCalls.h"
#include "revng/Support/IRHelpers.h"

llvm::PreservedAnalyses
RemoveHelperCallsPass::run(llvm::Function &F,
                           llvm::FunctionAnalysisManager &FAM) {
  using namespace llvm;
  using GCBI = GeneratedCodeBasicInfo;

  SmallVector<Instruction *, 16> ToReplace;
  for (auto &BB : F)
    for (auto &I : BB)
      if (isCallToHelper(&I))
        ToReplace.push_back(&I);

  bool Changed = not ToReplace.empty();
  if (!Changed)
    return PreservedAnalyses::all();

  OpaqueFunctionsPool<Type *> OFPOriginalHelper(F.getParent(), false);
  OpaqueFunctionsPool<Type *> OFPRegsClobberedHelper(F.getParent(), false);

  OFPOriginalHelper.addFnAttribute(Attribute::ReadOnly);
  OFPOriginalHelper.addFnAttribute(Attribute::NoUnwind);

  OFPRegsClobberedHelper.addFnAttribute(Attribute::ReadOnly);
  OFPRegsClobberedHelper.addFnAttribute(Attribute::NoUnwind);

  IRBuilder<> Builder(F.getContext());
  for (auto *I : ToReplace) {
    Builder.SetInsertPoint(I);
    auto UsedCSVs = std::move(GCBI::getCSVUsedByHelperCall(I).Written);

    auto *FRetTy = cast<CallInst>(I)->getFunctionType()->getReturnType();
    auto *OrigHelpOpqF = OFPOriginalHelper.get(FRetTy,
                                               FRetTy,
                                               {},
                                               "original_helper");
    CallInst *NewHelper = Builder.CreateCall(OrigHelpOpqF);

    for (auto *CSV : UsedCSVs) {
      auto *CSVTy = CSV->getType()->getPointerElementType();
      auto *RegsClobbOpqF = OFPRegsClobberedHelper.get(CSVTy,
                                                       CSVTy,
                                                       {},
                                                       "regs_clobbered_helper");
      Builder.CreateStore(Builder.CreateCall(RegsClobbOpqF), CSV);
    }

    I->replaceAllUsesWith(NewHelper);
    I->eraseFromParent();
  }

  return PreservedAnalyses::none();
}
