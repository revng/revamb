#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Value.h"

#include "revng/Support/BlockType.h"
#include "revng/Support/IRHelpers.h"

inline llvm::IntegerType *getCSVType(llvm::GlobalVariable *CSV) {
  using namespace llvm;
  return cast<IntegerType>(CSV->getType()->getPointerElementType());
}

namespace NextJumpTarget {

enum Values { Unique, Multiple, Helper };

};

namespace PCAffectingCSV {

enum Values { PC, IsThumb };

};

namespace detail {

using namespace llvm;

using CSVFactory = std::function<GlobalVariable *(PCAffectingCSV::Values CSVID,
                                                  StringRef Name)>;

}; // namespace detail

using CSVFactory = detail::CSVFactory;

class ProgramCounterHandler {
protected:
  static constexpr const char *AddressName = "pc";
  static constexpr const char *AddressSpaceName = "pc_address_space";
  static constexpr const char *EpochName = "pc_epoch";
  static constexpr const char *TypeName = "pc_type";

protected:
  llvm::GlobalVariable *AddressCSV;
  llvm::GlobalVariable *EpochCSV;
  llvm::GlobalVariable *AddressSpaceCSV;
  llvm::GlobalVariable *TypeCSV;

  std::set<llvm::Value *> CSVsAffectingPC;

public:
  using DispatcherTarget = std::pair<MetaAddress, llvm::BasicBlock *>;
  using DispatcherTargets = std::vector<DispatcherTarget>;

protected:
  ProgramCounterHandler() :
    AddressCSV(nullptr),
    EpochCSV(nullptr),
    AddressSpaceCSV(nullptr),
    TypeCSV(nullptr) {}

public:
  virtual ~ProgramCounterHandler() {}

public:
  static std::unique_ptr<ProgramCounterHandler>
  create(llvm::Triple::ArchType Architecture,
         llvm::Module *M,
         const CSVFactory &Factory);

  static std::unique_ptr<ProgramCounterHandler>
  fromModule(llvm::Triple::ArchType Architecture, llvm::Module *M);

public:
  std::array<llvm::GlobalVariable *, 4> pcCSVs() const {
    return { EpochCSV, AddressSpaceCSV, TypeCSV, AddressCSV };
  }

  /// \brief Hook for the emission of a store to a CSV
  ///
  /// \param Builder IRBuilder to employ in order to inject new instructions. It
  ///        is positioned after \p Store
  /// \param Store the StoreInst targeting a CSV
  ///
  /// \return true if new instructions have been emitted.
  bool handleStore(llvm::IRBuilder<> &Builder, llvm::StoreInst *Store) const {
    if (affectsPC(Store))
      return handleStoreInternal(Builder, Store);
    return false;
  }

  void initializePC(llvm::IRBuilder<> &Builder, MetaAddress NewPC) const {
    setPC(Builder, NewPC);
    initializePCInternal(Builder, NewPC);
  }

  void setPC(llvm::IRBuilder<> &Builder, MetaAddress NewPC) const {
    revng_assert(NewPC.isValid() and NewPC.isCode());
    store(Builder, AddressCSV, NewPC.address());
    store(Builder, EpochCSV, NewPC.epoch());
    store(Builder, AddressSpaceCSV, NewPC.addressSpace());
    store(Builder, TypeCSV, NewPC.type());
  }

  void expandNewPC(llvm::CallInst *Call) const {
    revng_assert(isCallTo(Call, "newpc"));
    auto MA = MetaAddress::fromConstant(Call->getArgOperand(0));
    llvm::IRBuilder<> Builder(Call);
    setPC(Builder, MA);
  }

  llvm::Value *loadPC(llvm::IRBuilder<> &Builder) const;

protected:
  virtual void
  initializePCInternal(llvm::IRBuilder<> &Builder, MetaAddress NewPC) const = 0;

  virtual bool handleStoreInternal(llvm::IRBuilder<> &Builder,
                                   llvm::StoreInst *Store) const = 0;

public:
  bool affectsPC(llvm::GlobalVariable *GV) const {
    return CSVsAffectingPC.count(GV) != 0;
  }

  bool affectsPC(llvm::StoreInst *Store) const {
    using namespace llvm;
    if (auto *CSV = dyn_cast<GlobalVariable>(Store->getPointerOperand()))
      return affectsPC(CSV);
    else
      return false;
  }

  /// \return an empty Optional if the PC has not changed on at least one path,
  ///         an invalid MetaAddress in case there isn't a single next PC, or,
  ///         finally, a valid MetaAddress representing the only possible next
  ///         PC
  std::pair<NextJumpTarget::Values, MetaAddress>
  getUniqueJumpTarget(llvm::BasicBlock *BB);

  void deserializePC(llvm::IRBuilder<> &Builder) const {
    using namespace llvm;

    // Load and re-store each CSV affecting the PC and then feed them to
    // handleStore
    for (Value *CSVAffectingPC : CSVsAffectingPC) {
      auto *FakeLoad = Builder.CreateLoad(CSVAffectingPC);
      auto *FakeStore = Builder.CreateStore(FakeLoad, CSVAffectingPC);
      bool HasInjectedCode = handleStore(Builder, FakeStore);
      FakeStore->eraseFromParent();

      if (not HasInjectedCode) {
        // The store did not produce any effect, the load is useless too
        revng_assert(FakeLoad->use_begin() == FakeLoad->use_end());
        FakeLoad->eraseFromParent();
      }
    }
  }

  virtual llvm::Value *loadJumpablePC(llvm::IRBuilder<> &Builder) const = 0;

  virtual std::array<llvm::Value *, 4>
  dissectJumpablePC(llvm::IRBuilder<> &Builder,
                    llvm::Value *ToDissect,
                    llvm::Triple::ArchType Arch) const = 0;

  virtual void
  deserializePCFromSignalContext(llvm::IRBuilder<> &Builder,
                                 llvm::Value *PCAddress,
                                 llvm::Value *SavedRegisters) const = 0;

  llvm::Instruction *composeIntegerPC(llvm::IRBuilder<> &B) const {
    return MetaAddress::composeIntegerPC(B,
                                         B.CreateLoad(AddressCSV),
                                         B.CreateLoad(EpochCSV),
                                         B.CreateLoad(AddressSpaceCSV),
                                         B.CreateLoad(TypeCSV));
  }

  bool isPCSizedType(llvm::Type *T) const {
    return T == AddressCSV->getType()->getPointerElementType();
  }

public:
  struct DispatcherInfo {
    llvm::SmallVector<llvm::BasicBlock *, 4> NewBlocks;
    llvm::SwitchInst *Switch;
  };

  /// \param Targets the targets to materialize for the dispatcher. Will be
  ///        sorted.
  DispatcherInfo
  buildDispatcher(DispatcherTargets &Targets,
                  llvm::IRBuilder<> &Builder,
                  llvm::BasicBlock *Default,
                  llvm::Optional<BlockType::Values> SetBlockType) const;

  DispatcherInfo
  buildDispatcher(DispatcherTargets &Targets,
                  llvm::BasicBlock *CreateIn,
                  llvm::BasicBlock *Default,
                  llvm::Optional<BlockType::Values> SetBlockType) const {
    llvm::IRBuilder<> Builder(CreateIn);
    return buildDispatcher(Targets, Builder, Default, SetBlockType);
  }

  /// \note \p Root must not already contain a case for \p NewTarget
  void
  addCaseToDispatcher(llvm::SwitchInst *Root,
                      const DispatcherTarget &NewTarget,
                      llvm::Optional<BlockType::Values> SetBlockType) const;

  void destroyDispatcher(llvm::SwitchInst *Root) const;

  void buildHotPath(llvm::IRBuilder<> &Builder,
                    const DispatcherTarget &CandidateTarget,
                    llvm::BasicBlock *Default) const;

protected:
  void createMissingVariables(llvm::Module *M) {
    if (AddressCSV == nullptr)
      AddressCSV = createAddress(M);
    if (EpochCSV == nullptr)
      EpochCSV = createEpoch(M);
    if (AddressSpaceCSV == nullptr)
      AddressSpaceCSV = createAddressSpace(M);
    if (TypeCSV == nullptr)
      TypeCSV = createType(M);
  }

public:
  void setMissingVariables(llvm::Module *M) {
    AddressCSV = M->getGlobalVariable(AddressName, true);
    EpochCSV = M->getGlobalVariable(EpochName, true);
    AddressSpaceCSV = M->getGlobalVariable(AddressSpaceName, true);
    TypeCSV = M->getGlobalVariable(TypeName, true);

    revng_assert(AddressCSV != nullptr and EpochCSV != nullptr
                 and AddressSpaceCSV != nullptr and TypeCSV != nullptr);
  }

private:
  bool isPCAffectingHelper(llvm::Instruction *I) const;

  static llvm::GlobalVariable *createAddress(llvm::Module *M) {
    return createVariable(M, AddressName, sizeof(MetaAddress::Address));
  }

  static llvm::GlobalVariable *createEpoch(llvm::Module *M) {
    return createVariable(M, EpochName, sizeof(MetaAddress::Epoch));
  }

  static llvm::GlobalVariable *createAddressSpace(llvm::Module *M) {
    return createVariable(M,
                          AddressSpaceName,
                          sizeof(MetaAddress::AddressSpace));
  }

  static llvm::GlobalVariable *createType(llvm::Module *M) {
    return createVariable(M, TypeName, sizeof(MetaAddress::Type));
  }

  static llvm::GlobalVariable *
  createVariable(llvm::Module *M, llvm::StringRef Name, size_t Size) {
    using namespace llvm;
    auto *T = Type::getIntNTy(M->getContext(), Size * 8);
    return new GlobalVariable(*M,
                              T,
                              false,
                              GlobalValue::ExternalLinkage,
                              ConstantInt::get(T, 0),
                              Name);
  }

protected:
  static llvm::StoreInst *
  store(llvm::IRBuilder<> &Builder, llvm::GlobalVariable *GV, uint64_t Value) {
    using namespace llvm;
    auto *Type = cast<IntegerType>(GV->getType()->getPointerElementType());
    return Builder.CreateStore(ConstantInt::get(Type, Value), GV);
  }
};
