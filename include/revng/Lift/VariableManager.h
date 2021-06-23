#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <map>
#include <string>

#include "llvm/IR/IRBuilder.h"
#include "llvm/Pass.h"

#include "revng/Support/CommandLine.h"
#include "revng/Support/revng.h"

#include "CPUStateAccessAnalysisPass.h"
#include "PTCDump.h"

namespace llvm {
class AllocaInst;
class BasicBlock;
class DataLayout;
class GlobalVariable;
class Module;
class StructType;
class Value;
} // namespace llvm

class VariableManager;

// TODO: rename
extern llvm::cl::opt<bool> External;

/// \brief Maintain the list of variables required by PTC
///
/// It can be queried for a variable, which, if not already existing, will be
/// created on the fly.
class VariableManager {
public:
  VariableManager(llvm::Module &M, Architecture &TargetArchitecture);

  void setAllocaInsertPoint(llvm::Instruction *I) {
    AllocaBuilder.SetInsertPoint(I);
  }

  llvm::Instruction *load(llvm::IRBuilder<> &Builder, unsigned TemporaryId) {
    using namespace llvm;

    auto [IsNew, V] = getOrCreate(TemporaryId, true);

    if (V == nullptr)
      return nullptr;

    if (IsNew) {
      auto *Undef = UndefValue::get(V->getType()->getPointerElementType());
      Builder.CreateStore(Undef, V);
    }

    return Builder.CreateLoad(V);
  }

  /// \brief Get or create the LLVM value associated to a PTC temporary
  ///
  /// Given a PTC temporary identifier, checks if it already exists in the
  /// generated LLVM IR, and, if not, it creates it.
  ///
  /// \param TemporaryId the PTC temporary identifier.
  ///
  /// \return a `Value` wrapping the requested global or local variable.
  llvm::Value *getOrCreate(unsigned TemporaryId) {
    return getOrCreate(TemporaryId, false).second;
  }

  /// \brief Return the global variable corresponding to \p Offset in the CPU
  ///        state.
  ///
  /// \param Offset the offset in the CPU state (the `env` PTC variable).
  /// \param Name an optional name to force for the associate global variable.
  ///
  /// \return a pair composed by the request global variable and the offset in
  ///         it corresponding to \p Offset. For instance, if you're accessing
  ///         the third byte of a 32-bit integer it will 2.
  std::pair<llvm::GlobalVariable *, unsigned>
  getByEnvOffset(intptr_t Offset, std::string Name = "") {
    return getByCPUStateOffsetInternal(EnvOffset + Offset, Name);
  }

  /// \brief Notify VariableManager to reset all the "function"-specific
  ///        information
  ///
  /// Informs the VariableManager that a new function has begun, so it can
  /// discard function- and basic block-level variables.
  ///
  /// Note: by "function" here we mean a function in PTC terms, i.e. a run of
  ///       code translated in a single shot by the TCG. Do not confuse this
  ///       function concept with other meanings.
  ///
  /// \param Instructions the new PTCInstructionList to use from now on.
  void newFunction(PTCInstructionList *Instructions);

  /// Informs the VariableManager that a new basic block has begun, so it can
  /// discard basic block-level variables.
  void newBasicBlock() { Temporaries.clear(); }

  /// Returns true if the given variable is the env variable
  bool isEnv(llvm::Value *TheValue);

  llvm::Value *computeEnvAddress(llvm::Type *TargetType,
                                 llvm::Instruction *InsertBefore,
                                 unsigned Offset = 0);

  void setDataLayout(const llvm::DataLayout *NewLayout) {
    ModuleLayout = NewLayout;
  }

  std::vector<llvm::AllocaInst *> locals() {
    std::vector<llvm::AllocaInst *> Locals;
    for (auto Pair : LocalTemporaries)
      Locals.push_back(Pair.second);
    return Locals;
  }

  llvm::Value *loadFromEnvOffset(llvm::IRBuilder<> &Builder,
                                 unsigned LoadSize,
                                 unsigned Offset) {
    return loadFromCPUStateOffset(Builder, LoadSize, EnvOffset + Offset);
  }

  llvm::Optional<llvm::StoreInst *> storeToEnvOffset(llvm::IRBuilder<> &Builder,
                                                     unsigned StoreSize,
                                                     unsigned Offset,
                                                     llvm::Value *ToStore) {
    unsigned ActualOffset = EnvOffset + Offset;
    return storeToCPUStateOffset(Builder, StoreSize, ActualOffset, ToStore);
  }

  bool memcpyAtEnvOffset(llvm::IRBuilder<> &Builder,
                         llvm::CallInst *CallMemcpy,
                         unsigned Offset,
                         bool EnvIsSrc);

  /// \brief Perform finalization steps on variables
  void finalize();

  void rebuildCSVList();

  /// \brief Gets the CPUStateType
  llvm::StructType *getCPUStateType() const { return CPUStateType; }

  bool hasEnv() const { return Env != nullptr; }

  llvm::Value *CPUStateToEnv(llvm::Value *CPUState,
                             llvm::Type *TargetType,
                             llvm::Instruction *InsertBefore) const;

private:
  std::pair<bool, llvm::Value *>
  getOrCreate(unsigned TemporaryId, bool Reading);

  llvm::Value *loadFromCPUStateOffset(llvm::IRBuilder<> &Builder,
                                      unsigned LoadSize,
                                      unsigned Offset);

  llvm::Optional<llvm::StoreInst *>
  storeToCPUStateOffset(llvm::IRBuilder<> &Builder,
                        unsigned StoreSize,
                        unsigned Offset,
                        llvm::Value *ToStore);

  llvm::GlobalVariable *
  getByCPUStateOffset(intptr_t Offset, std::string Name = "");

  std::pair<llvm::GlobalVariable *, unsigned>
  getByCPUStateOffsetInternal(intptr_t Offset, std::string Name = "");

private:
  llvm::Module &TheModule;
  llvm::IRBuilder<> AllocaBuilder;
  using TemporariesMap = std::map<unsigned int, llvm::AllocaInst *>;
  using GlobalsMap = std::map<intptr_t, llvm::GlobalVariable *>;
  GlobalsMap CPUStateGlobals;
  GlobalsMap OtherGlobals;
  TemporariesMap Temporaries;
  TemporariesMap LocalTemporaries;
  PTCInstructionList *Instructions;

  llvm::StructType *CPUStateType;
  const llvm::DataLayout *ModuleLayout;
  unsigned EnvOffset;

  llvm::GlobalVariable *Env;
  Architecture &TargetArchitecture;
};
