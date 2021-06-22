#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <algorithm>
#include <optional>
#include <string>

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/Pass.h"
#include "llvm/Support/DOTGraphTraits.h"
#include "llvm/Support/GraphWriter.h"

#include "revng/ABIAnalyses/ABIAnalysis.h"
#include "revng/ADT/GenericGraph.h"
#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/FunctionCallIdentification/FunctionCallIdentification.h"
#include "revng/Model/Binary.h"
#include "revng/Model/LoadModelPass.h"
#include "revng/StackAnalysis/AAWriterPass.h"
#include "revng/StackAnalysis/FunctionsSummary.h"
#include "revng/StackAnalysis/IndirectBranchInfoPrinterPass.h"
#include "revng/StackAnalysis/PromoteGlobalToLocalVars.h"
#include "revng/StackAnalysis/SegregateDirectStackAccesses.h"
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

struct FunctionSummary {
  FunctionSummary(
    model::FunctionType::Values Type,
    const std::set<llvm::GlobalVariable *> &ClobberedRegisters,
    std::optional<uint64_t> ElectedFSO,
    llvm::Function *FakeFunction) :
    Type(Type),
    ClobberedRegisters(ClobberedRegisters),
    ElectedFSO(ElectedFSO),
    FakeFunction(FakeFunction) {}

  FunctionSummary() = delete;
  FunctionSummary(const FunctionSummary &) = delete;
  FunctionSummary(FunctionSummary &&) = default;
  FunctionSummary &operator=(const FunctionSummary &) = delete;
  FunctionSummary &operator=(FunctionSummary &&) = default;

  model::FunctionType::Values Type;
  std::set<llvm::GlobalVariable *> ClobberedRegisters;
  std::optional<uint64_t> ElectedFSO;
  llvm::Function *FakeFunction;

  static bool compare(const FunctionSummary &Old, const FunctionSummary &New) {
    if (New.Type == Old.Type)
      return std::includes(Old.ClobberedRegisters.begin(),
                           Old.ClobberedRegisters.end(),
                           New.ClobberedRegisters.begin(),
                           New.ClobberedRegisters.end());

    return (New.Type <= Old.Type);
  }

  void dump() const debug_function { dump(dbg); }

  template<typename T>
  void dump(T &Output) const {
    Output << "Dumping summary of the function. \n"
           << "Type: " << Type << "\n"
           << "ElectedFSO: " << *ElectedFSO << "\n"
           << "Clobbered registers: ";

    for (auto *Reg : ClobberedRegisters)
      Output << ::getName(Reg);
    Output << "\n";
  }
};

class FunctionAnalysisResults {
private:
  /// \brief Map from CFEP to its function description
  std::map<llvm::BasicBlock *, FunctionSummary> FunctionsBucket;
  FunctionSummary DefaultSummary;

  const FunctionSummary &get(llvm::BasicBlock *BB) const {
    auto It = FunctionsBucket.find(BB);
    if (It != FunctionsBucket.end())
      return It->second;
    return DefaultSummary;
  }

public:
  FunctionAnalysisResults(FunctionSummary DefaultSummary) :
    DefaultSummary(std::move(DefaultSummary)) {}

  model::FunctionType::Values getFunctionType(llvm::BasicBlock *BB) const {
    return get(BB).Type;
  }

  bool isFakeFunction(llvm::BasicBlock *BB) const {
    return getFunctionType(BB) == model::FunctionType::Values::Fake;
  }

  llvm::Function *getFakeFunction(llvm::BasicBlock *BB) const {
    return get(BB).FakeFunction;
  }

  const auto &getRegistersClobbered(llvm::BasicBlock *BB) const {
    return get(BB).ClobberedRegisters;
  }

  std::optional<uint64_t> getElectedFSO(llvm::BasicBlock *BB) const {
    return get(BB).ElectedFSO;
  }

  bool registerFunction(llvm::BasicBlock *BB, FunctionSummary &&F) {
    auto It = FunctionsBucket.find(BB);
    if (It != FunctionsBucket.end()) {
      bool Changed = FunctionSummary::compare(It->second, F);
      It->second = std::move(F);
      return !(Changed);
    }
    FunctionsBucket.try_emplace(BB, std::move(F));
    return true;
  }
};

template<class FunctionOracle>
class CFEPAnalyzer {
  llvm::Module &M;
  llvm::LLVMContext &Context;
  GeneratedCodeBasicInfo *GCBI;
  FunctionOracle &Oracle;
  llvm::Function *PreHookMarker;
  llvm::Function *PostHookMarker;
  llvm::Function *RetHookMarker;
  llvm::Function *IndirectBranchInfoMarker;
  OpaqueFunctionsPool<llvm::StringRef> OFPRegistersClobbered;

public:
  CFEPAnalyzer(llvm::Module &M,
               GeneratedCodeBasicInfo *GCBI,
               FunctionOracle &Oracle,
               llvm::Function *PreHookMarker,
               llvm::Function *PostHookMarker,
               llvm::Function *RetHookMarker) :
    M(M),
    Context(M.getContext()),
    GCBI(GCBI),
    Oracle(Oracle),
    PreHookMarker(PreHookMarker),
    PostHookMarker(PostHookMarker),
    RetHookMarker(RetHookMarker),
    OFPRegistersClobbered(&M, false) {}

public:
  FunctionSummary
  analyze(const std::vector<llvm::GlobalVariable *> &, llvm::BasicBlock *BB);

private:
  llvm::Function *createDisposableFunction(llvm::BasicBlock *BB);
  llvm::BasicBlock *integrateFunctionCallee(llvm::BasicBlock *BB);
  void throwDisposableFunction(llvm::Function *F);
  FunctionSummary milkResults(const std::vector<llvm::GlobalVariable *> &,
                              ABIAnalyses::ABIAnalysesResults &,
                              const std::set<llvm::GlobalVariable *> &,
                              llvm::Function *F);
};

} // namespace StackAnalysis

struct BasicBlockNodeData {
  BasicBlockNodeData(llvm::BasicBlock *BB) : BB(BB){};
  llvm::BasicBlock *BB;
};
using BasicBlockNode = BidirectionalNode<BasicBlockNodeData>;
using SmallCallGraph = GenericGraph<BasicBlockNode>;

template<>
struct llvm::DOTGraphTraits<SmallCallGraph *>
  : public llvm::DefaultDOTGraphTraits {
  using EdgeIterator = llvm::GraphTraits<SmallCallGraph *>::ChildIteratorType;
  DOTGraphTraits(bool IsSimple = false) : DefaultDOTGraphTraits(IsSimple) {}

  static std::string
  getNodeLabel(const BasicBlockNode *Node, const SmallCallGraph *Graph) {
    if (Node->BB == nullptr)
      return "null";
    return Node->BB->getName().str();
  }

  static std::string getEdgeAttributes(const BasicBlockNode *Node,
                                       const EdgeIterator EI,
                                       const SmallCallGraph *Graph) {
    return "color=black,style=dashed";
  }
};
