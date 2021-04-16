#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <string>

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/Pass.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/FunctionCallIdentification/FunctionCallIdentification.h"
#include "revng/Model/LoadModelPass.h"
#include "revng/StackAnalysis/FunctionsSummary.h"
#include "revng/Support/OpaqueFunctionsPool.h"

namespace StackAnalysis {

extern const std::set<llvm::GlobalVariable *> EmptyCSVSet;

class StackAnalysis : public llvm::ModulePass {
  friend class FunctionBoundariesDetectionPass;

public:
  static char ID;

public:
  StackAnalysis() : llvm::ModulePass(ID) {}

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    AU.addRequired<GeneratedCodeBasicInfoWrapperPass>();
    AU.addRequired<LoadModelWrapperPass>();
  }

  bool runOnModule(llvm::Module &M) override;

  const std::set<llvm::GlobalVariable *> &
  getClobbered(llvm::BasicBlock *Function) const {
    auto It = GrandResult.Functions.find(Function);
    if (It == GrandResult.Functions.end())
      return EmptyCSVSet;
    else
      return It->second.ClobberedRegisters;
  }

  void serialize(std::ostream &Output) { Output << TextRepresentation; }

  void serializeMetadata(llvm::Function &F, GeneratedCodeBasicInfo &GCBI);

public:
  FunctionsSummary GrandResult;
  std::string TextRepresentation;
};

struct FuncSummary {
  FuncSummary(
    model::FunctionType::Values Type,
    std::set<llvm::GlobalVariable *> ClobberedRegisters,
    llvm::SmallSet<std::pair<llvm::CallInst *, model::FunctionEdgeType::Values>,
                   4> Result,
    std::optional<uint64_t> ElectedFSO,
    llvm::Function *FakeFunction = nullptr) :
    Type(Type),
    ClobberedRegisters(ClobberedRegisters),
    Result(Result),
    ElectedFSO(ElectedFSO),
    FakeFunction(FakeFunction) {}

  model::FunctionType::Values Type;
  std::set<llvm::GlobalVariable *> ClobberedRegisters;
  llvm::SmallSet<std::pair<llvm::CallInst *, model::FunctionEdgeType::Values>,
                 4>
    Result;
  std::optional<uint64_t> ElectedFSO;
  llvm::Function *FakeFunction;
};

class FunctionProperties {
private:
  /// \brief Map from CFEP to its function description
  llvm::DenseMap<llvm::BasicBlock *, FuncSummary> Bucket;

public:
  FunctionProperties() {}

  model::FunctionType::Values getFunctionType(llvm::BasicBlock *BB) {
    return model::FunctionType::Values::Regular;
  }

  llvm::Function *getFakeFunction(llvm::BasicBlock *BB) {
    auto It = Bucket.find(BB);
    if (It != Bucket.end())
      return It->second.FakeFunction;
    return nullptr;
  }

  const auto &getRegistersClobbered(llvm::BasicBlock *BB) const {
    auto It = Bucket.find(BB);
    if (It != Bucket.end())
      return It->second.ClobberedRegisters;
    return EmptyCSVSet;
  }

  void registerFunc(llvm::BasicBlock *BB, FuncSummary F) {
    Bucket.try_emplace(BB, F);
  }
};

template<class FunctionOracle>
class CFEPAnalyzer {
  llvm::Module &M;
  llvm::LLVMContext &Context;
  GeneratedCodeBasicInfo *GCBI;
  FunctionOracle &Oracle;
  OpaqueFunctionsPool<llvm::StringRef> OFPRegistersClobbered;
  OpaqueFunctionsPool<llvm::StringRef> OFPIndirectBranchInfo;
  OpaqueFunctionsPool<llvm::StringRef> OFPHooksFunctionCall;
  llvm::SmallVector<llvm::CallInst *, 4> IndirectBranchInfoCalls;

public:
  CFEPAnalyzer(llvm::Module &M,
               GeneratedCodeBasicInfo *GCBI,
               FunctionOracle &Oracle) :
    M(M),
    Context(M.getContext()),
    GCBI(GCBI),
    Oracle(Oracle),
    OFPRegistersClobbered(&M, false),
    OFPIndirectBranchInfo(&M, false),
    OFPHooksFunctionCall(&M, false) {}

public:
  FuncSummary
  analyze(const std::vector<llvm::GlobalVariable *> &, llvm::BasicBlock *BB);

private:
  llvm::Function *createDisposableFunction(llvm::BasicBlock *BB);
  llvm::BasicBlock *integrateFunctionCallee(llvm::BasicBlock *BB);
  void throwDisposableFunction(llvm::Function *F);
  FuncSummary milkResults(const std::vector<llvm::GlobalVariable *> &,
                          llvm::Function *F);
};

} // namespace StackAnalysis
