/// \file BitLiveness.cpp
/// \brief In this file we model the transfer functions for the analysis.
/// Each transfer function models the information flow of a function or of
/// a special case of an instruction.
/// In our case, `R = transferXyz(Ins, E)` means that for the instruction Ins
/// if we assume that the first E bits of the result of the instruction
/// are alive, then the first R bits of the operands are also alive

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <limits>

#include "llvm/ADT/APInt.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Casting.h"

#include "revng/Support/Assert.h"
#include "revng/TypeShrinking/BitLiveness.h"
#include "revng/TypeShrinking/DataFlowGraph.h"
#include "revng/MFP/MFP.h"

namespace TypeShrinking {

/// This class is an instance of monotone framework
/// the elements represent the index from which all bits are not alive
/// so for an element E, all bits with index < E are alive
struct BitLivenessAnalysis {
  using GraphType = GenericGraph<DataFlowNode> *;
  using LatticeElement = uint32_t;
  using Label = DataFlowNode *;
  using MFPResult = MFP::MFPResult<BitLivenessAnalysis::LatticeElement>;

  uint32_t combineValues(const uint32_t &LHS, const uint32_t &RHS) const {
    return std::max(LHS, RHS);
  }

  bool isLessOrEqual(const uint32_t &LHS, const uint32_t &RHS) const {
    return LHS <= RHS;
  }

  uint32_t applyTransferFunction(DataFlowNode *L, const uint32_t E) const;
};

using BitVector = llvm::BitVector;
using Instruction = llvm::Instruction;

llvm::AnalysisKey TypeShrinking::BitLivenessPass::Key;
char TypeShrinking::BitLivenessWrapperPass::ID = 0;

static llvm::RegisterPass<BitLivenessWrapperPass> X("bit-liveness",
                                                    "Apply approximate bit "
                                                    "liveness analysis",
                                                    true,
                                                    true);

const uint32_t Top = std::numeric_limits<uint32_t>::max();

bool isDataFlowSink(const Instruction *Ins) {
  if (Ins->mayHaveSideEffects()) {
    return true;
  }
  switch (Ins->getOpcode()) {
  case Instruction::Call:
  case Instruction::CallBr:
  case Instruction::Ret:
  case Instruction::Store:
  case Instruction::Br:
  case Instruction::IndirectBr:
    return true;
  default:
    return false;
  }
}

static uint32_t getMaxOperandSize(Instruction *Ins) {
  uint32_t Max = 0;
  for (auto &Operand : Ins->operands()) {
    if (Operand->getType()->isIntegerTy())
      Max = std::max(Max, Operand->getType()->getIntegerBitWidth());
    else
      return Top;
  }
  return Max;
}

/// If Ins has an integer return type return its bitwidth
/// else return top
static uint32_t getResultSize(Instruction *Ins) {
  uint32_t Size = Top;
  if (Ins->getType()->isIntegerTy()) {
    Size = std::min(Size, Ins->getType()->getIntegerBitWidth());
  }
  return Size;
}

/// A specialization of the transfer function for the and instruction
/// In cases where one of the operands is a constant mask
///
/// example:
///   `%1 = %0 & 0xff`
///
/// only the lower 8 bits of `%0` flow into `%1`, but if only the lower 4 bits
/// of `%1` flow into a data flow sink, then only the lower 4 bits of `%0`
/// will flow into the data flow sink
static uint32_t
transferMask(const uint32_t &Element, const uint32_t &MaskIndex) {
  return std::min(Element, MaskIndex);
}

/// Transfer function for the and instruction
///
/// example:
///   `%2 = %1 & %0
///
/// if none of the operands are constants
/// then liveness of %1 and %0 = liveness of %2
static uint32_t transferAnd(Instruction *Ins, const uint32_t &Element) {
  revng_assert(Ins->getOpcode() == Instruction::And);
  uint32_t Result = Element;
  for (auto &Operand : Ins->operands()) {
    if (auto *ConstantOperand = llvm::dyn_cast<llvm::ConstantInt>(Operand)) {
      auto OperandValue = ConstantOperand->getUniqueInteger();
      auto MostSignificantBit = OperandValue.getBitWidth()
                                - OperandValue.countLeadingZeros();
      Result = std::min(Result, transferMask(Element, MostSignificantBit));
    }
  }
  return Result;
}

/// Transfer function for the left shift instruction
///
/// example:
///   `%2 = %1 << %0
///
/// if none of the operands are constants
/// then every bit of %1 and %0 can be alive
///
/// if %0 is a constant, then the first E bits of %2 are the first E - %0
/// bits of %1 padded with zeros
static uint32_t transferShiftLeft(Instruction *Ins, const uint32_t &Element) {
  uint32_t OperandSize = getMaxOperandSize(Ins);
  if (auto ConstOp = llvm::dyn_cast<llvm::ConstantInt>(Ins->getOperand(1))) {
    auto OpVal = ConstOp->getZExtValue();
    if (Element < OpVal)
      return 0;
    return Element - OpVal;
  }
  return OperandSize;
}

/// Transfer function for the logical right shift instruction
///
/// example:
///   `%2 = %1 >>L %0
///
/// if none of the operands are constants
/// then every bit of %1 and %0 can be alive
///
/// if %0 is a constant, then the first E bits of %2 come from the first E +
/// %0 bits of %1
static uint32_t
transferLogicalShiftRight(Instruction *Ins, const uint32_t &Element) {
  uint32_t OperandSize = getMaxOperandSize(Ins);
  if (auto ConstOp = llvm::dyn_cast<llvm::ConstantInt>(Ins->getOperand(1))) {
    auto OpVal = ConstOp->getZExtValue();
    revng_assert(OpVal < Top);
    if (Top - OpVal < Element)
      return Top;
    return std::min(OperandSize, Element + (uint32_t) OpVal);
  }
  return OperandSize;
}

/// Transfer function for the arithmetical right shift instruction
///
/// example:
///   `%2 = %1 >>A %0
///
/// if none of the operands are constants
/// then every bit of %1 and %0 can be alive
///
/// if %0 is a constant, then the first E bits of %2 come from the first E +
/// %0 bits of %1
static uint32_t
transferArithmeticalShiftRight(Instruction *Ins, const uint32_t &Element) {
  uint32_t OperandSize = getMaxOperandSize(Ins);
  if (auto ConstOp = llvm::dyn_cast<llvm::ConstantInt>(Ins->getOperand(1))) {
    auto OpVal = ConstOp->getZExtValue();
    revng_assert(OpVal < Top);
    if (Top - OpVal < Element)
      return Top;
    return std::min(OperandSize, Element + (uint32_t) OpVal);
  }
  return OperandSize;
}

/// Transfer function for the trunc instruction
///
/// example:
///   `%2 = truncX(%1)
///
/// at most the lower X bits of %1 flow into %2
static uint32_t transferTrunc(Instruction *Ins, const uint32_t &Element) {
  return std::min(Element, Ins->getType()->getIntegerBitWidth());
}

/// Transfer function for the trunc instruction
///
/// example:
///   `%2 = zext(%1)
///
/// at most all the bits in %1 flow into %2
static uint32_t transferZExt(Instruction *Ins, const uint32_t &Element) {
  return std::min(Element, getMaxOperandSize(Ins));
}

uint32_t
BitLivenessAnalysis::applyTransferFunction(DataFlowNode *L, const uint32_t E) const {
  auto *Ins = L->Instruction;
  uint32_t Input = E;
  // At most every bit of the result is alive
  if (!isDataFlowSink(Ins)) {
    Input = std::min(E, getResultSize(Ins));
  }
  switch (Ins->getOpcode()) {
  case Instruction::And:
    return transferAnd(Ins, E);
  case Instruction::Xor:
  case Instruction::Or:
  case Instruction::Add:
  case Instruction::Sub:
  case Instruction::Mul:
    return std::min(E, getMaxOperandSize(L->Instruction));
  case Instruction::Shl:
    return transferShiftLeft(Ins, E);
  case Instruction::LShr:
    return transferLogicalShiftRight(Ins, E);
  case Instruction::AShr:
    return transferArithmeticalShiftRight(Ins, E);
  case Instruction::Trunc:
    return transferTrunc(Ins, E);
  case Instruction::ZExt:
    return transferZExt(Ins, E);
  default:
    // By default all the bits of the operands can be alive
    return Top;
  }
}

BitLivenessPass::Result
BitLivenessPass::run(llvm::Function &F, llvm::FunctionAnalysisManager &) {
  GenericGraph<DataFlowNode> DataFlowGraph = buildDataFlowGraph(F);
  std::vector<DataFlowNode *> ExtremalLabels;
  for (DataFlowNode *Node : DataFlowGraph.nodes()) {
    if (isDataFlowSink(Node->Instruction)) {
      ExtremalLabels.push_back(Node);
    }
  }

  auto MFPResults = MFP::getMaximalFixedPoint<BitLivenessAnalysis>({},
                                                              &DataFlowGraph,
                                                              0,
                                                              Top,
                                                              ExtremalLabels);
  BitLivenessPass::Result Result;
  for (auto &[Label, MFPResult] : MFPResults)
    Result[Label->Instruction] = MFPResult.OutValue;

  return Result;
}

bool BitLivenessWrapperPass::runOnFunction(llvm::Function &F) {
  llvm::FunctionAnalysisManager FAM;
  Result = BitLivenessPass().run(F, FAM);
  return false;
}

} // namespace TypeShrinking
