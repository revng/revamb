#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Analysis/LazyValueInfo.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/IR/Dominators.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Support/KnownBits.h"

#include "revng/BasicAnalyses/AdvancedValueInfo.h"

#include "JumpTargetManager.h"

inline Logger<> AVIPassLogger("avipass");

class StaticDataMemoryOracle {
private:
  const llvm::DataLayout &DL;
  JumpTargetManager &JTM;
  BinaryFile::Endianess E;

public:
  StaticDataMemoryOracle(const llvm::DataLayout &DL, JumpTargetManager &JTM) :
    DL(DL), JTM(JTM) {
    // Read the value using the endianess of the destination architecture,
    // since, if there's a mismatch, in the stack we will also have a byteswap
    // instruction
    using Endianess = BinaryFile::Endianess;
    E = (DL.isLittleEndian() ? Endianess::LittleEndian : Endianess::BigEndian);
  }

  const llvm::DataLayout &getDataLayout() const { return DL; }

  MaterializedValue load(llvm::Constant *Address) {
    return JTM.readFromPointer(Address, E);
  }
};

class AdvancedValueInfoPass
  : public llvm::PassInfoMixin<AdvancedValueInfoPass> {
private:
  JumpTargetManager *JTM;
  static constexpr const char *MarkerName = "revng_avi";

public:
  AdvancedValueInfoPass(JumpTargetManager *JTM) : JTM(JTM) {}

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &);

  static llvm::Function *createMarker(llvm::Module *M) {
    using namespace llvm;
    using FT = FunctionType;
    LLVMContext &C = getContext(M);
    FT *Type = FT::get(FT::getVoidTy(C), {}, true);
    FunctionCallee Callee = M->getOrInsertFunction(MarkerName, Type);
    auto *Marker = cast<Function>(Callee.getCallee());
    Marker->addFnAttr(llvm::Attribute::InaccessibleMemOnly);
    return Marker;
  }
};

inline llvm::PreservedAnalyses
AdvancedValueInfoPass::run(llvm::Function &F,
                           llvm::FunctionAnalysisManager &FAM) {
  using namespace llvm;

  std::set<Instruction *> ToReplace;
  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      if (I.getOpcode() == Instruction::Or) {
        auto &DL = F.getParent()->getDataLayout();

        Value *LHS = I.getOperand(0);
        Value *RHS = I.getOperand(1);
        const APInt &LHSZeros = computeKnownBits(LHS, DL).Zero;
        const APInt &RHSZeros = computeKnownBits(RHS, DL).Zero;

        if ((~RHSZeros & ~LHSZeros).isNullValue()) {
          ToReplace.insert(&I);
        }
      }
    }
  }

  for (Instruction *I : ToReplace) {
    I->replaceAllUsesWith(BinaryOperator::Create(Instruction::Add,
                                                 I->getOperand(0),
                                                 I->getOperand(1),
                                                 Twine(),
                                                 I));
    I->eraseFromParent();
  }

  Function *Marker = F.getParent()->getFunction(MarkerName);
  if (Marker == nullptr)
    return llvm::PreservedAnalyses::all();

  // The StaticDataMemoryOracle provide the contents of memory areas that are
  // mapped statically (i.e., in segments). This is critical to capture, e.g.,
  // virtual tables
  StaticDataMemoryOracle MO(F.getParent()->getDataLayout(), *JTM);

  auto &LVI = FAM.getResult<LazyValueAnalysis>(F);
  auto &DT = FAM.getResult<DominatorTreeAnalysis>(F);

  BasicBlock *Entry = &F.getEntryBlock();
  SwitchInst *Terminator = cast<SwitchInst>(Entry->getTerminator());
  BasicBlock *Dispatcher = Terminator->getDefaultDest();

  auto &SCEV = FAM.getResult<ScalarEvolutionAnalysis>(F);
  AdvancedValueInfo<StaticDataMemoryOracle> AVI(LVI, SCEV, DT, MO, Dispatcher);

#ifndef NDEBUG
  // Ensure that no instruction has itself as operand, except for phis
  for (BasicBlock &BB : F)
    for (Instruction &I : BB)
      for (Value *V : I.operands())
        revng_assert(isa<PHINode>(V) or V != &I);
#endif

  for (User *U : Marker->users()) {
    auto *Call = dyn_cast<CallInst>(U);
    if (Call == nullptr or skipCasts(Call->getCalledOperand()) != Marker
        or Call->getParent()->getParent() != &F)
      continue;

    revng_assert(Call->getNumArgOperands() >= 1);
    Value *ToTrack = Call->getArgOperand(0);

    AVIPassLogger << "Tracking " << ToTrack << ":";

    // Let AVI provide a series of possible values
    MaterializedValues Values = AVI.explore(Call->getParent(), ToTrack);

    //
    // Create a revng.avi metadata containing the type of instruction and
    // all the possible values we identified
    //
    QuickMetadata QMD(getContext(&F));
    std::vector<Metadata *> ValuesMD;
    ValuesMD.reserve(Values.size());
    for (const MaterializedValue &V : Values) {
      // TODO: we are we ignoring those with symbols
      if (not V.hasSymbol()) {
        ValuesMD.push_back(QMD.get(V.value()));
      }
    }

    Call->setMetadata("revng.avi", QMD.tuple(ValuesMD));

    if (AVIPassLogger.isEnabled()) {
      for (const MaterializedValue &V : Values) {
        AVIPassLogger << " ";
        V.dump(AVIPassLogger);
      }
      AVIPassLogger << DoLog;
    }
  }

  return PreservedAnalyses::all();
}
