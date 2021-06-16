#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"

#include "revng/Model/Binary.h"

inline const char *ModelMetadataName = "revng.model";

TupleTree<model::Binary> loadModel(const llvm::Module &M);

class ModelWrapper {
private:
  TupleTree<model::Binary> TheBinary;
  bool HasChanged = false;

public:
  ModelWrapper(TupleTree<model::Binary> &&TheBinary) :
    TheBinary(std::move(TheBinary)) {}

public:
  const model::Binary &getReadOnlyModel() const { return *TheBinary; }

  TupleTree<model::Binary> &getWriteableModel() {
    HasChanged = true;
    return TheBinary;
  }

  bool hasChanged() const { return HasChanged; }

  template<typename IRUnitT, typename PreservedAnalysesT, typename InvalidatorT>
  bool invalidate(IRUnitT &, const PreservedAnalysesT &, InvalidatorT &) {
    return false;
  }
};

class LoadModelWrapperPass : public llvm::ImmutablePass {
public:
  static char ID;

private:
  std::optional<ModelWrapper> Wrapper;

public:
  LoadModelWrapperPass() : llvm::ImmutablePass(ID) {}

public:
  bool doInitialization(llvm::Module &M) override final;
  bool doFinalization(llvm::Module &M) override final;

public:
  ModelWrapper &get() { return *Wrapper; }
};

class LoadModelAnalysis : public llvm::AnalysisInfoMixin<LoadModelAnalysis> {
  friend llvm::AnalysisInfoMixin<LoadModelAnalysis>;
  static llvm::AnalysisKey Key;

public:
  using Result = ModelWrapper;

  Result run(llvm::Module &M, llvm::ModuleAnalysisManager &);
  Result run(llvm::Function &F, llvm::FunctionAnalysisManager &);
};
