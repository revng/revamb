#ifndef VARIABLEMANAGER_H
#define VARIABLEMANAGER_H

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

// Standard includes
#include <cstdint>
#include <map>
#include <string>

// LLVM includes
#include "llvm/IR/IRBuilder.h"
#include "llvm/Pass.h"

// Local libraries includes
#include "revng/Support/CommandLine.h"
#include "revng/Support/revng.h"

// Local includes
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
class CPUStateAccessAnalysisPass;

// TODO: rename
extern llvm::cl::opt<bool> External;

/// \brief Maintain the list of variables required by PTC
///
/// It can be queried for a variable, which, if not already existing, will be
/// created on the fly.
class VariableManager {
public:
  VariableManager(llvm::Module &TheModule,
                  llvm::Module &HelpersModule,
                  Architecture &TargetArchitecture);

  /// \brief Get or create the LLVM value associated to a PTC temporary
  ///
  /// Given a PTC temporary identifier, checks if it already exists in the
  /// generated LLVM IR, and, if not, it creates it.
  ///
  /// \param TemporaryId the PTC temporary identifier.
  ///
  /// \return a `Value` wrapping the requested global or local variable.
  // TODO: rename to getByTemporaryId
  llvm::Value *getOrCreate(unsigned TemporaryId, bool Reading);

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
  /// \param Delimiter the new point where to insert allocations for local
  ///        variables.
  /// \param Instructions the new PTCInstructionList to use from now on.
  void newFunction(llvm::Instruction *Delimiter = nullptr,
                   PTCInstructionList *Instructions = nullptr);

  /// Informs the VariableManager that a new basic block has begun, so it can
  /// discard basic block-level variables.
  ///
  /// \param Delimiter the new point where to insert allocations for local
  /// variables.
  /// \param Instructions the new PTCInstructionList to use from now on.
  void newBasicBlock(llvm::Instruction *Delimiter = nullptr,
                     PTCInstructionList *Instructions = nullptr);

  void newBasicBlock(llvm::BasicBlock *Delimiter,
                     PTCInstructionList *Instructions = nullptr);

  /// Returns true if the given variable is the env variable
  bool isEnv(llvm::Value *TheValue);

  CPUStateAccessAnalysisPass *createCPUStateAccessAnalysisPass() {
    return new CPUStateAccessAnalysisPass(this);
  }

  llvm::Value *computeEnvAddress(llvm::Type *TargetType,
                                 llvm::Instruction *InsertBefore,
                                 unsigned Offset = 0);

  void setDataLayout(const llvm::DataLayout *NewLayout) {
    ModuleLayout = NewLayout;
  }

  template<typename T>
  T *setAliasScope(T *Instruction);

  template<typename T>
  T *setNoAlias(T *Instruction);

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

  bool storeToEnvOffset(llvm::IRBuilder<> &Builder,
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
  void finalize() {
    using namespace llvm;

    if (not External) {
      for (auto &P : CPUStateGlobals)
        P.second->setLinkage(GlobalValue::InternalLinkage);
      for (auto &P : OtherGlobals)
        P.second->setLinkage(GlobalValue::InternalLinkage);
    }

    LLVMContext &Context = getContext(&TheModule);
    IRBuilder<> Builder(Context);

    // Create the setRegister function
    auto *SetRegisterTy = FunctionType::get(Builder.getVoidTy(),
                                            { Builder.getInt32Ty(),
                                              Builder.getInt64Ty() },
                                            false);
    auto *Temp = TheModule.getOrInsertFunction("set_register", SetRegisterTy);
    auto *SetRegister = cast<Function>(Temp);
    SetRegister->setLinkage(GlobalValue::ExternalLinkage);

    // Collect arguments
    auto ArgIt = SetRegister->arg_begin();
    auto ArgEnd = SetRegister->arg_end();
    revng_assert(ArgIt != ArgEnd);
    Argument *RegisterID = &*ArgIt;
    ArgIt++;
    revng_assert(ArgIt != ArgEnd);
    Argument *NewValue = &*ArgIt;
    ArgIt++;
    revng_assert(ArgIt == ArgEnd);

    // Create main basic blocks
    using BasicBlock = BasicBlock;
    auto *EntryBB = BasicBlock::Create(Context, "", SetRegister);
    auto *DefaultBB = BasicBlock::Create(Context, "", SetRegister);
    auto *ReturnBB = BasicBlock::Create(Context, "", SetRegister);

    // Populate the default case of the switch
    Builder.SetInsertPoint(DefaultBB);
    Builder.CreateCall(TheModule.getFunction("abort"));
    Builder.CreateUnreachable();

    // Create the switch statement
    Builder.SetInsertPoint(EntryBB);
    auto *Switch = Builder.CreateSwitch(RegisterID,
                                        DefaultBB,
                                        CPUStateGlobals.size());
    for (auto &P : CPUStateGlobals) {
      Type *CSVTy = P.second->getType();
      auto *CSVIntTy = cast<IntegerType>(CSVTy->getPointerElementType());
      if (CSVIntTy->getBitWidth() <= 64) {
        // Set the value of the CSV
        auto *SetRegisterBB = BasicBlock::Create(Context, "", SetRegister);
        Builder.SetInsertPoint(SetRegisterBB);
        Builder.CreateStore(Builder.CreateTrunc(NewValue, CSVIntTy), P.second);
        Builder.CreateBr(ReturnBB);

        // Add the case to the switch
        Switch->addCase(Builder.getInt32(P.first), SetRegisterBB);
      }
    }

    // Finally, populate the return basic block
    Builder.SetInsertPoint(ReturnBB);
    Builder.CreateRetVoid();
  }

  /// \brief Gets the CPUStateType
  llvm::StructType *getCPUStateType() const { return CPUStateType; }

private:
  llvm::Value *loadFromCPUStateOffset(llvm::IRBuilder<> &Builder,
                                      unsigned LoadSize,
                                      unsigned Offset);

  bool storeToCPUStateOffset(llvm::IRBuilder<> &Builder,
                             unsigned StoreSize,
                             unsigned Offset,
                             llvm::Value *ToStore);

  llvm::GlobalVariable *
  getByCPUStateOffset(intptr_t Offset, std::string Name = "");

  std::pair<llvm::GlobalVariable *, unsigned>
  getByCPUStateOffsetInternal(intptr_t Offset, std::string Name = "");

private:
  llvm::Module &TheModule;
  llvm::IRBuilder<> Builder;
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

  llvm::Value *Env;
  unsigned AliasScopeMDKindID;
  unsigned NoAliasMDKindID;
  llvm::MDNode *CPUStateScopeSet;

  Architecture &TargetArchitecture;
};

#endif // VARIABLEMANAGER_H