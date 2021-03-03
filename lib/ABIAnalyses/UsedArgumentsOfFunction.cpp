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

#include "revng/ABIAnalyses/Generated/UsedArgumentsOfFunction.h"
#include "revng/MFP/MFP.h"
#include "revng/Support/revng.h"

namespace UsedArgumentsOfFunction {
using namespace llvm;

DenseMap<GlobalVariable *, State>
analyze(const Instruction *CallSite,
        const BasicBlock *Entry,
        const GeneratedCodeBasicInfo &GCBI,
        const StackAnalysis::FunctionProperties &FP) {

  MFI<true> Instance{ { GCBI } };
  DenseMap<Register, CoreLattice::LatticeElement> InitialValue{};
  DenseMap<Register, CoreLattice::LatticeElement> ExtremalValue{};

  auto Results = MFP::getMaximalFixedPoint<MFI<true>>(Instance,
                                                Entry,
                                                InitialValue,
                                                ExtremalValue,
                                                { Entry },
                                                { Entry });

  DenseMap<GlobalVariable *, State> RegYes{};

  for (auto &[BB, Result] : Results) {
    for (auto &[RegID, RegState] : Result.OutValue) {
      if (RegState == CoreLattice::Yes) {
        if (auto *GV = Instance.getABIRegister(RegID)) {
          RegYes[GV] = State::Yes;
        }
      }
    }
  }

  return RegYes;
}
} // namespace UsedArgumentsOfFunction