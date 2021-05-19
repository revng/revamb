#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <array>
#include <memory>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Pass.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/ValueMapper.h"

#include "revng/AutoEnforcer/BackingContainerRegistry.h"
#include "revng/AutoEnforcer/InputOutputContract.h"
#include "revng/AutoEnforcer/Step.h"
#include "revng/Support/Debug.h"

namespace AutoEnforcer {

class LLVMContainer {
public:
  LLVMContainer(std::unique_ptr<llvm::Module> M) : Module(std::move(M)) {
    revng_assert(Module != nullptr);
  }

  void mergeBackDerived(LLVMContainer &ToMerge) {
    llvm::Linker TheLinker(*Module);
    bool Result = TheLinker.linkInModule(std::move(ToMerge.Module),
                                         llvm::Linker::OverrideFromSrc);
    revng_assert(!Result, "Linker failed");
  }

  const llvm::Module &getModule() const { return *Module; }
  llvm::Module &getModule() { return *Module; }

protected:
  std::unique_ptr<llvm::Module> Module;
};

class DefaultLLVMContainer : public BackingContainer<DefaultLLVMContainer> {
public:
  const static char ID;
  using TargetContainer = BackingContainersStatus::TargetContainer;

  DefaultLLVMContainer(std::unique_ptr<llvm::Module> M) :
    BackingContainer<DefaultLLVMContainer>(), Container(std::move(M)) {}

  bool contains(const AutoEnforcerTarget &Target) const override {
    const auto &LastName = Target.getQuantifiers().back().getName();
    return Container.getModule().getFunction(LastName) != nullptr;
  }

  void mergeBackDerived(DefaultLLVMContainer &ToMerge) override {
    Container.mergeBackDerived(ToMerge.Container);
  }

  const llvm::Module &getModule() const { return Container.getModule(); }
  llvm::Module &getModule() { return Container.getModule(); }

  std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargetContainer &Targets) const override;

  static bool classof(const BackingContainerBase *Base) {
    return Base->isA<DefaultLLVMContainer>();
  }

  bool remove(const AutoEnforcerTarget &Target) final {
    const auto &Name = Target.getQuantifiers().back().getName();
    const auto &GlobalSymbol = getModule().getNamedGlobal(Name);
    if (not GlobalSymbol)
      return false;
    GlobalSymbol->eraseFromParent();
    return true;
  }

private:
  LLVMContainer Container;
};

template<typename LLVMContainerType>
class LLVMContainerFactory : public BackingContainerRegistryEntry {
public:
  LLVMContainerFactory(llvm::LLVMContext &Context) : Context(Context) {}

  std::unique_ptr<BackingContainerBase> createEmpty() const override {
    auto Module = std::make_unique<llvm::Module>("rev.ng module", Context);
    return std::make_unique<LLVMContainerType>(std::move(Module));
  }

private:
  llvm::LLVMContext &Context;
};

using DefaultLLVMContainerFactory = LLVMContainerFactory<DefaultLLVMContainer>;

namespace Detail {
class LLVMEnforcerBaseImpl {
public:
  virtual ~LLVMEnforcerBaseImpl() = default;
  virtual void registerPassess(llvm::legacy::PassManager &Manager) = 0;
  virtual llvm::ArrayRef<InputOutputContract> getContract() const = 0;
  virtual std::unique_ptr<LLVMEnforcerBaseImpl> clone() const = 0;
  virtual llvm::StringRef getName() const = 0;
};

template<typename LLVMEnforcerPass>
class LLVMEnforcerImpl : public LLVMEnforcerBaseImpl {
public:
  using RegistrationFunctionType = void (*)(llvm::legacy::PassManager &);

  ~LLVMEnforcerImpl() override = default;
  void registerPassess(llvm::legacy::PassManager &Manager) override {
    EnforcerPass.registerPassess(Manager);
  }

  llvm::ArrayRef<InputOutputContract> getContract() const override {
    return Contract;
  }

  std::unique_ptr<LLVMEnforcerBaseImpl> clone() const override {
    return std::make_unique<LLVMEnforcerImpl>(*this);
  }

  LLVMEnforcerImpl(LLVMEnforcerPass Pass) : EnforcerPass(std::move(Pass)) {
    for (const auto &C : EnforcerPass.getContract())
      Contract.push_back(C);
  }

  llvm::StringRef getName() const override { return LLVMEnforcerPass::Name; }

private:
  LLVMEnforcerPass EnforcerPass;
  llvm::SmallVector<InputOutputContract, 2> Contract;
};

} // namespace Detail

class LLVMEnforcer {

public:
  static constexpr auto Name = "LLVMEnforcer";
  template<typename... LLVMEnforcerPass>
  explicit LLVMEnforcer(LLVMEnforcerPass... Pass) {
    (emplacePass<LLVMEnforcerPass>(std::move(Pass)), ...);
  }

  LLVMEnforcer &operator=(const LLVMEnforcer &Other);
  LLVMEnforcer(const LLVMEnforcer &Other);
  LLVMEnforcer &operator=(LLVMEnforcer &&Other) = default;
  LLVMEnforcer(LLVMEnforcer &&Other) = default;
  ~LLVMEnforcer() = default;

  llvm::SmallVector<InputOutputContract, 3> getContract() const;
  void run(DefaultLLVMContainer &Container);

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    for (const auto &Pass : Passess) {
      indent(OS, Indents);
      OS << Pass->getName().str() << "\n";
    }
  }

  void dump() const { dump(dbg); }

private:
  template<typename LLVMEnforcerPass>
  void emplacePass(LLVMEnforcerPass Pass) {
    using Type = Detail::LLVMEnforcerImpl<LLVMEnforcerPass>;
    auto Wrapper = std::make_unique<Type>(std::forward<LLVMEnforcerPass>(Pass));
    Passess.emplace_back(std::move(Wrapper));
  }

  llvm::SmallVector<std::unique_ptr<Detail::LLVMEnforcerBaseImpl>, 3> Passess;
};

template<typename... LLVMEnforcerPassess>
EnforcerWrapper
wrapLLVMPassess(std::string LLVMModuleName, LLVMEnforcerPassess &&... P) {
  return EnforcerWrapper(LLVMEnforcer(std::move(P)...),
                         { std::move(LLVMModuleName) });
}

} // namespace AutoEnforcer
