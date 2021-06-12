/// \file SegregateDirectStackAccesses.cpp
/// \brief Segregate direct stack accesses from all other memory accesses
/// through alias information. This provides a way to say that stack accesses do
/// not interfere with any other memory access.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/PatternMatch.h"

#include "revng/StackAnalysis/SegregateDirectStackAccesses.h"
#include "revng/Support/Assert.h"

using namespace llvm;
using SDSAP = SegregateDirectStackAccessesPass;

PreservedAnalyses SDSAP::run(Function &F, FunctionAnalysisManager &FAM) {
  Context = &(F.getContext());

  // Get the result of the GCBI analysis
  auto &M = *F.getParent();
  const auto &MAMProxy = FAM.getResult<ModuleAnalysisManagerFunctionProxy>(F);
  GCBI = MAMProxy.getCachedResult<GeneratedCodeBasicInfoAnalysis>(M);
  if (!GCBI)
    GCBI = &(FAM.getResult<GeneratedCodeBasicInfoAnalysis>(F));
  revng_assert(GCBI != nullptr);

  // Populate the two buckets with all load and store instruction of the
  // function, properly segregated.
  segregateAccesses(F);

  // Adorn IR with the alias information collected before.
  decorateStackAccesses();

  return PreservedAnalyses::none();
}

void SDSAP::segregateAccesses(Function &F) {
  using namespace PatternMatch;
  Value *LoadSP = nullptr;
  bool Found = false;

  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      if (auto *LI = dyn_cast<LoadInst>(&I)) {
        if (GCBI->isSPReg(skipCasts(LI->getPointerOperand()))) {
          LoadSP = LI;
          Found = true;
          break;
        }
      }
    }

    if (Found)
      break;
  }

  // Context: inttoptr instructions basically inhibits all optimizations. In
  // particular, when an integer is inttoptr'd twice with different destination
  // type, alias analysis messes up. Hence, we need to ensure that no inttoptr
  // exists when operating on a instruction that directly accesses the stack.
  // Note that this problem will be addressed by opaque pointers in the future.
  IRBuilder<> Builder(&F.getEntryBlock().front());
  auto *I8PtrTy = Builder.getInt8PtrTy();
  Value *SPI8Ptr = Builder
                     .CreateLoad(I8PtrTy,
                                 ConstantExpr::getBitCast(GCBI->spReg(),
                                                          I8PtrTy
                                                            ->getPointerTo()));

  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      Value *Ptr = nullptr;

      // Differentiate accesses and add them onto their respective bucket.
      // Everything that is not a direct access on the stack is put onto the
      // bucket `NotDirectStackAccesses`. Load/store that access the CSVs will
      // have their alias info added later as well.
      if (auto *LI = dyn_cast<LoadInst>(&I))
        Ptr = skipCasts(LI->getPointerOperand());
      else if (auto *SI = dyn_cast<StoreInst>(&I))
        Ptr = skipCasts(SI->getPointerOperand());

      if (Ptr) {
        Value *LoadPtr, *LHS, *BitCast;
        ConstantInt *Offset;
        Builder.SetInsertPoint(&I);

        // Do we have a inttoptr to a load i64, i64* LoadPtr as pointer operand
        // of the current instruction, where LoadPtr is SP? Change it with the
        // newly-created bitcasted load in order to prevent from using inttoptr.
        if (match(Ptr, m_Load(m_Value(LoadPtr)))
            && GCBI->isSPReg(skipCasts(LoadPtr))) {
          BitCast = Builder.CreateBitCast(SPI8Ptr,
                                          isa<LoadInst>(&I) ?
                                            I.getOperand(0)->getType() :
                                            I.getOperand(0)
                                              ->getType()
                                              ->getPointerTo());
          I.setOperand(isa<LoadInst>(&I) ? 0 : 1, BitCast);

          DirectStackAccesses.emplace_back(&I);

          // Do we have a inttoptr to a add i64 LHS, -8 as pointer operand of
          // the current instruction, where LHS is SP? Canonicalize the i2p +
          // add into a gep whose value is bitcasted to the original type of SP.
        } else if (match(Ptr, m_c_Add(m_Value(LHS), m_ConstantInt(Offset)))
                   && LHS == LoadSP) {
          auto *GEP = Builder.CreateGEP(Builder.getInt8Ty(), SPI8Ptr, Offset);
          BitCast = Builder.CreateBitCast(GEP,
                                          isa<LoadInst>(&I) ?
                                            I.getOperand(0)->getType() :
                                            I.getOperand(0)
                                              ->getType()
                                              ->getPointerTo());
          I.setOperand(isa<LoadInst>(&I) ? 0 : 1, BitCast);

          DirectStackAccesses.emplace_back(&I);
        } else {
          NotDirectStackAccesses.emplace_back(&I);
        }
      }
    }
  }
}

void SDSAP::decorateStackAccesses() {
  MDBuilder MDB(*Context);

  MDNode *AliasDomain = MDB.createAliasScopeDomain("CSVAliasDomain");

  auto *DSAScope = MDB.createAliasScope("DirectStackAccessScope", AliasDomain);
  auto *NDSAScope = MDB.createAliasScope("Not(DirectStackAccessScope)",
                                         AliasDomain);

  auto *DSASet = MDNode::get(*Context, ArrayRef<Metadata *>({ DSAScope }));
  auto *NDSASet = MDNode::get(*Context, ArrayRef<Metadata *>({ NDSAScope }));

  for (auto *I : DirectStackAccesses) {
    I->setMetadata(LLVMContext::MD_alias_scope, DSASet);
    I->setMetadata(LLVMContext::MD_noalias, NDSASet);
  }

  for (auto *I : NotDirectStackAccesses) {
    I->setMetadata(LLVMContext::MD_alias_scope, NDSASet);
    I->setMetadata(LLVMContext::MD_noalias, DSASet);
  }
}
