//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/GraphTraits.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Casting.h"

#include "revng/MFP/MFP.h"
#include "revng/Support/revng.h"

#include "Analyses.h"

namespace ABIAnalyses::UsedReturnValuesOfFunction {
using namespace llvm;
using namespace ABIAnalyses;

DenseMap<const GlobalVariable *, State>
analyze(const BasicBlock *ReturnBlock, const GeneratedCodeBasicInfo &GCBI) {
  using MFI = MFIAnalysis<false, CoreLattice>;

  MFI Instance{ { GCBI } };
  MFI::LatticeElement InitialValue;
  MFI::LatticeElement ExtremalValue(CoreLattice::ExtremalLatticeElement);
  auto Results = MFP::
    getMaximalFixedPoint<MFI, MFI::GT, MFI::LGT>(Instance,
                                                 ReturnBlock,
                                                 InitialValue,
                                                 ExtremalValue,
                                                 { ReturnBlock },
                                                 { ReturnBlock });

  DenseSet<const GlobalVariable *> RegUnknown{};
  DenseMap<const GlobalVariable *, State> RegYesOrDead{};

  for (auto &[BB, Result] : Results) {
    for (auto &[GV, RegState] : Result.OutValue) {
      if (RegState == CoreLattice::Unknown) {
        RegUnknown.insert(GV);
      }
    }
  }

  for (auto &[BB, Result] : Results) {
    for (auto &[GV, RegState] : Result.OutValue) {
      if (RegState == CoreLattice::YesOrDead && RegUnknown.count(GV) == 0) {
        RegYesOrDead[GV] = State::YesOrDead;
      }
    }
  }

  return RegYesOrDead;
}
} // namespace ABIAnalyses::UsedReturnValuesOfFunction
