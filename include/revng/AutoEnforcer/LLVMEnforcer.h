#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <array>
#include <memory>
#include <system_error>
#include <utility>
#include <vector>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Analysis/ScalarEvolutionExpressions.h"
#include "llvm/Bitcode/BitcodeReader.h"
#include "llvm/Bitcode/BitcodeWriter.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Pass.h"
#include "llvm/Support/Error.h"
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

    // WIP: drop body of stuff that it's already there

    bool Result = TheLinker.linkInModule(std::move(ToMerge.Module),
                                         llvm::Linker::OverrideFromSrc);
    revng_assert(!Result, "Linker failed");
  }

  const llvm::Module &getModule() const { return *Module; }
  llvm::Module &getModule() { return *Module; }

  llvm::Error storeToDisk(llvm::StringRef Path) const {
    std::error_code EC;
    llvm::raw_fd_ostream OS(Path, EC, llvm::sys::fs::F_None);
    if (EC)
      return llvm::createStringError(EC,
                                     "Could not store to file module %s",
                                     Path.str().c_str());
    llvm::WriteBitcodeToFile(*Module, OS);
    OS.flush();
    return llvm::Error::success();
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) {
    llvm::SMDiagnostic Error;
    auto M = llvm::parseIRFile(Path, Error, Module->getContext());
    if (!M)
      return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                     "Could not parse file %s",
                                     Path.str().c_str());

    Module = std::move(M);
    return llvm::Error::success();
  }

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

  void mergeBackDerived(DefaultLLVMContainer &&ToMerge) override {
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

  llvm::Error storeToDisk(llvm::StringRef Path) const override {
    return Container.storeToDisk(Path);
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) override {
    return Container.loadFromDisk(Path);
  }

private:
  LLVMContainer Container;
};

template<typename LLVMContainerType>
class LLVMContainerFactory : public BackingContainerFactory {
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

class LLVMEnforcerBaseImpl {
public:
  virtual ~LLVMEnforcerBaseImpl() = default;
  virtual void registerPassess(llvm::legacy::PassManager &Manager) = 0;
  virtual const std::vector<AtomicContract> &getContract() const = 0;
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

  const std::vector<AtomicContract> &getContract() const override {
    return Contract;
  }

  std::unique_ptr<LLVMEnforcerBaseImpl> clone() const override {
    return std::make_unique<LLVMEnforcerImpl>(*this);
  }

  LLVMEnforcerImpl(LLVMEnforcerPass Pass) :
    EnforcerPass(std::move(Pass)), Contract(this->EnforcerPass.getContract()) {}

  llvm::StringRef getName() const override { return LLVMEnforcerPass::Name; }

private:
  LLVMEnforcerPass EnforcerPass;
  std::vector<AtomicContract> Contract;
};

class LLVMEnforcer {

public:
  static constexpr auto Name = "LLVMEnforcer";
  template<typename... LLVMEnforcerPass>
  explicit LLVMEnforcer(LLVMEnforcerPass... Pass) {
    (addPass<LLVMEnforcerPass>(std::move(Pass)), ...);
  }

  LLVMEnforcer &operator=(const LLVMEnforcer &Other);
  LLVMEnforcer(const LLVMEnforcer &Other);
  LLVMEnforcer &operator=(LLVMEnforcer &&Other) = default;
  LLVMEnforcer(LLVMEnforcer &&Other) = default;
  ~LLVMEnforcer() = default;

  std::vector<AtomicContract> getContract() const;
  void run(DefaultLLVMContainer &Container);

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const debug_function {
    for (const auto &Pass : Passess) {
      indent(OS, Indents);
      OS << Pass->getName().str() << "\n";
    }
  }

  void dump() const debug_function { dump(dbg); }

  template<typename LLVMEnforcerPass>
  void addPass(LLVMEnforcerPass Pass) {
    using Type = LLVMEnforcerImpl<LLVMEnforcerPass>;
    auto Wrapper = std::make_unique<Type>(std::forward<LLVMEnforcerPass>(Pass));
    Passess.emplace_back(std::move(Wrapper));
  }

  void addPass(std::unique_ptr<LLVMEnforcerBaseImpl> Impl) {
    Passess.emplace_back(std::move(Impl));
  }

private:
  llvm::SmallVector<std::unique_ptr<LLVMEnforcerBaseImpl>, 3> Passess;
};

class PureLLVMEnforcer {
public:
  static constexpr auto Name = "PureLLVMEnforcer";

  std::vector<AtomicContract> getContract() const { return {}; }
  void run(DefaultLLVMContainer &Container);

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const debug_function {
    for (const auto &Pass : PassNames) {
      indent(OS, Indents);
      OS << Pass << "\n";
    }
  }

  static llvm::Expected<PureLLVMEnforcer>
  create(std::vector<std::string> PassNames) {
    for (const auto &Name : PassNames)
      if (llvm::PassRegistry::getPassRegistry()->getPassInfo(Name) == nullptr)
        return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                       "Could not load llvm pass %s ",
                                       Name.c_str());

    return PureLLVMEnforcer(std::move(PassNames));
  }

  PureLLVMEnforcer() = default;
  ~PureLLVMEnforcer() = default;
  PureLLVMEnforcer(PureLLVMEnforcer &&) = default;
  PureLLVMEnforcer(const PureLLVMEnforcer &) = default;
  PureLLVMEnforcer &operator=(PureLLVMEnforcer &&) = default;
  PureLLVMEnforcer &operator=(const PureLLVMEnforcer &) = default;

private:
  PureLLVMEnforcer(std::vector<std::string> Names) :
    PassNames(std::move(Names)) {}
  std::vector<std::string> PassNames;
};

template<typename... LLVMEnforcerPassess>
EnforcerWrapper
wrapLLVMPassess(std::string LLVMModuleName, LLVMEnforcerPassess &&... P) {
  return EnforcerWrapper(LLVMEnforcer(std::move(P)...),
                         { std::move(LLVMModuleName) });
}

} // namespace AutoEnforcer
