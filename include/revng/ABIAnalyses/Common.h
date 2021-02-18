#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/IR/Value.h"
#include "llvm/Support/Casting.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/MFP/MFP.h"
#include "revng/Model/Binary.h"
#include "revng/StackAnalysis/StackAnalysis.h"
#include "revng/Support/revng.h"

namespace ABIAnalyses {

using Register = model::Register::Values;

enum TransferKind {
  Read,
  Write,
  WeakWrite,
  TheCall,
  None,
};

struct ABIAnalysis {
private:
  llvm::SmallSet<llvm::GlobalVariable *, 20> ABIRegisters;
  llvm::SmallVector<llvm::GlobalVariable *, 20> RegisterList;
  const llvm::Instruction *CallSite;

public:
  ABIAnalysis(const GeneratedCodeBasicInfo &GCBI) :
    ABIAnalysis(nullptr, GCBI){};

  ABIAnalysis(const llvm::Instruction *CS, const GeneratedCodeBasicInfo &GCBI) :
    RegisterList(), CallSite(CS) {

    for (auto *CSV : GCBI.abiRegisters()) {
      if (CSV) {
        ABIRegisters.insert(CSV);
        RegisterList.emplace_back(CSV);
      }
    }
  };

  const llvm::SmallVector<llvm::GlobalVariable *, 20> getRegisters() const {
    return RegisterList;
  }

  bool isABIRegister(const llvm::Value *) const;

  TransferKind classifyInstruction(const llvm::Instruction *) const;

  llvm::SmallVector<const llvm::GlobalVariable *, 1>
  getRegistersWritten(const llvm::Instruction *) const;

  llvm::SmallVector<const llvm::GlobalVariable *, 1>
  getRegistersRead(const llvm::Instruction *) const;
};

inline bool ABIAnalysis::isABIRegister(const llvm::Value *V) const {
  using namespace llvm;
  if (auto *G = dyn_cast<GlobalVariable>(V)) {
    return ABIRegisters.count(G) != 0;
  }
  return false;
}

inline bool isCallSiteBlock(const llvm::BasicBlock *B) {
  using namespace llvm;
  if (auto *C = dyn_cast<CallInst>(&*B->getFirstInsertionPt())) {
    if (C->getCalledFunction()->getName().contains("precall_hook")) {
      return true;
    }
  }
  return false;
}

inline const llvm::Instruction *getPreCallHook(const llvm::BasicBlock *B) {
  if (isCallSiteBlock(B)) {
    return &*B->getFirstInsertionPt();
  }
  return nullptr;
}

inline const llvm::Instruction *getPostCallHook(const llvm::BasicBlock *B) {
  if (isCallSiteBlock(B)) {
    return B->getTerminator()->getPrevNode();
  }
  return nullptr;
}

inline TransferKind
ABIAnalysis::classifyInstruction(const llvm::Instruction *I) const {
  using namespace llvm;
  switch (I->getOpcode()) {
  case Instruction::Store: {
    auto *S = cast<StoreInst>(I);
    if (isABIRegister(S->getPointerOperand())) {
      return isCallSiteBlock(I->getParent()) ? WeakWrite : Write;
    }
    break;
  }
  case Instruction::Load: {
    auto *L = cast<LoadInst>(I);
    if (isABIRegister(L->getPointerOperand())) {
      return Read;
    }
    break;
  }
  case Instruction::Call: {
    if (I == CallSite) {
      return TheCall;
    }
    break;
  }
  }
  return None;
}

inline llvm::SmallVector<const llvm::GlobalVariable *, 1>
ABIAnalysis::getRegistersWritten(const llvm::Instruction *I) const {
  using namespace llvm;
  SmallVector<const GlobalVariable *, 1> Result;
  switch (I->getOpcode()) {
  case Instruction::Store: {
    auto *S = cast<StoreInst>(I);
    auto *Pointer = S->getPointerOperand();
    if (isABIRegister(Pointer)) {
      Result.push_back(cast<GlobalVariable>(Pointer));
    }
    break;
  }
  }
  return Result;
}

inline llvm::SmallVector<const llvm::GlobalVariable *, 1>
ABIAnalysis::getRegistersRead(const llvm::Instruction *I) const {
  using namespace llvm;
  SmallVector<const GlobalVariable *, 1> Result;
  switch (I->getOpcode()) {
  case Instruction::Load: {
    auto *L = cast<LoadInst>(I);
    auto *Pointer = L->getPointerOperand();
    if (isABIRegister(Pointer)) {
      Result.push_back(cast<GlobalVariable>(Pointer));
    }
    break;
  }
  }
  return Result;
}

template<typename CoreLattice>
struct RegistersMap : public llvm::DenseMap<const llvm::GlobalVariable *,
                                            typename CoreLattice::LatticeElement> {
private:
  using InnerLatticeElement = typename CoreLattice::LatticeElement;
  using Base = llvm::DenseMap<const llvm::GlobalVariable *,
                              InnerLatticeElement>;

private:
  InnerLatticeElement Default {};

public:
  RegistersMap() : Default {} {}
  RegistersMap(InnerLatticeElement Default) : Default(Default) {}

public:
  InnerLatticeElement getOrDefault(const llvm::GlobalVariable *GV) const {
    auto It = Base::find(GV);
    if (It == Base::end())
      return Default;
    else
      return It->second;
  }

public:
  RegistersMap combine(const RegistersMap &RHS) const {
    RegistersMap New = *this;
    for (const auto &[Reg, S] : RHS) {
      auto LHSValue = New.getOrDefault(Reg);
      auto RHSValue = RHS.getOrDefault(Reg);
      New[Reg] = CoreLattice::combineValues(LHSValue, RHSValue);
    }

    New.Default = CoreLattice::combineValues(New.Default, RHS.Default);

    return New;
  }

  bool isLessOrEqual(const RegistersMap &RHS) const {
    const RegistersMap &LHS = *this;

    if (!CoreLattice::isLessOrEqual(LHS.Default, RHS.Default))
      return false;

    for (auto &[Reg, S] : LHS) {
      auto LHSValue = LHS.getOrDefault(Reg);
      auto RHSValue = RHS.getOrDefault(Reg);
      if (!CoreLattice::isLessOrEqual(LHSValue, RHSValue)) {
        return false;
      }
    }
    for (auto &[Reg, S] : RHS) {
      auto LHSValue = LHS.getOrDefault(Reg);
      auto RHSValue = RHS.getOrDefault(Reg);
      if (!CoreLattice::isLessOrEqual(LHSValue, RHSValue)) {
        return false;
      }
    }
    return true;
  }
};

template<bool IsForward, typename CoreLattice>
struct MFIAnalysis : ABIAnalyses::ABIAnalysis {
  using LatticeElement = RegistersMap<CoreLattice>;
  using Label = const llvm::BasicBlock *;
  using GraphType = std::conditional_t<IsForward,
                                       const llvm::BasicBlock *,
                                       llvm::Inverse<const llvm::BasicBlock *>>;
  using GT = llvm::GraphTraits<GraphType>;
  using LGT = GraphType;

  LatticeElement
  combineValues(const LatticeElement &LHS, const LatticeElement &RHS) const {
    return LHS.combine(RHS);
  };

  bool
  isLessOrEqual(const LatticeElement &LHS, const LatticeElement &RHS) const {
    return LHS.isLessOrEqual(RHS);
  };

  LatticeElement applyTransferFunction(Label L, const LatticeElement &E) const {
    using namespace llvm;
    LatticeElement New = E;
    std::vector<const Instruction *> InsList;
    for (auto &I : make_range(L->begin(), L->end())) {
      InsList.push_back(&I);
    }
    for (size_t i = 0; i < InsList.size(); i++) {
      auto I = InsList[IsForward ? i : (InsList.size() - i - 1)];
      TransferKind T = classifyInstruction(I);
      switch (T) {
      case TheCall: {
        for (auto &Reg : getRegisters()) {
          auto RegState = New.getOrDefault(Reg);
          New[Reg] = CoreLattice::transfer(TheCall, RegState);
        }
        break;
      }
      case Read:
        for (auto &Reg : getRegistersRead(I)) {
          auto RegState = New.getOrDefault(Reg);
          New[Reg] = CoreLattice::transfer(T, RegState);
        }
        break;
      case WeakWrite:
      case Write:
        for (auto &Reg : getRegistersWritten(I)) {
          auto RegState = New.getOrDefault(Reg);
          New[Reg] = CoreLattice::transfer(T, RegState);
        }
        break;
      default:
        break;
      }
    }
    return New;
  };
};

} // namespace ABIAnalyses
