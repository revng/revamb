/// \file TypeShrinking.cpp
/// \brief This analysis finds which bits of each Instruction is alive

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <set>
#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"

#include "revng/TypeShrinking/BitLiveness.h"
#include "revng/TypeShrinking/DataFlowGraph.h"
#include "revng/TypeShrinking/TypeShrinking.h"

using namespace llvm;

using BitSet = std::set<int>;

static cl::opt<uint32_t> MinimumWidth("min-width",
                                      cl::init(8),
                                      cl::desc("ignore analysis results for "
                                               "width lower than"),
                                      cl::value_desc("min-width"),
                                      cl::cat(MainCategory));

char TypeShrinking::TypeShrinkingWrapperPass::ID = 0;

using Register = RegisterPass<TypeShrinking::TypeShrinkingWrapperPass>;
static Register
  X("type-shrinking", "Run the type shrinking analysis", true, true);

namespace TypeShrinking {

void TypeShrinkingWrapperPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<BitLivenessWrapperPass>();
}

/// Returns true if each bit B of the result of Ins depends only on
/// the bits of the operands with an index lower than B
static bool isAddLike(const Instruction *Ins) {
  switch (Ins->getOpcode()) {
  case llvm::Instruction::And:
  case llvm::Instruction::Xor:
  case llvm::Instruction::Or:
  case llvm::Instruction::Add:
  case llvm::Instruction::Sub:
  case llvm::Instruction::Mul:
    return true;
  }
  return false;
}

static bool
runTypeShrinking(Function &F, const BitLivenessAnalysisResults &FixedPoints) {
  bool HasChanges = false;

  const std::array<uint32_t, 4> Ranks = { 8, 16, 32, 64 };
  for (auto &[Ins, Result] : FixedPoints) {
    // Find the closest rank that contains all the alive bits.
    // If there is a known rank and this is an instruction that behaves like add
    // (the least significant bits of the result depend only on the least
    // significant bits of the operands) we can down cast the operands and then
    // upcast the result
    if (Result >= MinimumWidth.getValue() && isAddLike(Ins)) {
      auto ClosestRank = std::lower_bound(Ranks.begin(), Ranks.end(), Result);
      if (ClosestRank != Ranks.end()
          && Ins->getType()->getScalarSizeInBits() > *ClosestRank) {
        auto Rank = *ClosestRank;
        HasChanges = true;
        llvm::Value *NewIns = nullptr;

        llvm::IRBuilder<> BuilderPre(Ins);
        llvm::IRBuilder<> BuilderPost(Ins->getNextNode());

        using CastOps = llvm::Instruction::CastOps;
        auto *Lhs = BuilderPre.CreateCast(CastOps::Trunc,
                                          Ins->getOperand(0),
                                          BuilderPre.getIntNTy(Rank));
        auto *Rhs = BuilderPre.CreateCast(CastOps::Trunc,
                                          Ins->getOperand(1),
                                          BuilderPre.getIntNTy(Rank));

        NewIns = BuilderPost.CreateBinOp((Instruction::BinaryOps)
                                           Ins->getOpcode(),
                                         Lhs,
                                         Rhs);

        // Emit ZExts, as late as possible
        SmallVector<std::pair<Use *, Value *>, 4> Replacements;
        for (Use &TheUse : Ins->uses()) {
          if (auto *U = cast<Instruction>(TheUse.getUser())) {
            IRBuilder<> B(U);

            // Fix insert point for PHIs
            if (auto *Phi = dyn_cast<PHINode>(U)) {
              auto *BB = Phi->getIncomingBlock(TheUse);
              auto It = BB->getTerminator()->getIterator();
              B.SetInsertPoint(BB, It);
            }

            auto *LateUpcast = B.CreateZExt(NewIns, Ins->getType());
            Replacements.emplace_back(&TheUse, LateUpcast);
          }
        }

        // Apply replacements
        for (auto &[Use, I] : Replacements)
          Use->set(I);

        // Drop the original instruction
        Ins->eraseFromParent();
      }
    }
  }

  return HasChanges;
}

bool TypeShrinkingWrapperPass::runOnFunction(Function &F) {
  auto &BitLiveness = getAnalysis<BitLivenessWrapperPass>();
  auto &FixedPoints = BitLiveness.getResult();
  return runTypeShrinking(F, FixedPoints);
}

PreservedAnalyses
TypeShrinkingPass::run(Function &F, FunctionAnalysisManager &FAM) {
  const auto &FixedPoints = FAM.getResult<BitLivenessPass>(F);
  bool HasChanges = runTypeShrinking(F, FixedPoints);
  return HasChanges ? PreservedAnalyses::none() : PreservedAnalyses::all();
}

} // namespace TypeShrinking
