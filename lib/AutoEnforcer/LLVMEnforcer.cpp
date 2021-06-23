//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/PassRegistry.h"
#include "llvm/PassSupport.h"

#include "revng/AutoEnforcer/LLVMEnforcer.h"

using namespace std;
using namespace llvm;
using namespace AutoEnforcer;

const char DefaultLLVMContainer::ID = '_';

unique_ptr<BackingContainerBase>
DefaultLLVMContainer::cloneFiltered(const TargetContainer &Targets) const {

  const auto MustBeCloned =
    [&Targets](const llvm::GlobalValue *Global) -> bool {
    const auto IsInObjectives = [Global](const AutoEnforcerTarget &Target) {
      return Target.getQuantifiers().back().getName() == Global->getName();
    };
    return find_if(Targets, IsInObjectives) != Targets.end();
  };

  ValueToValueMapTy Map;
  auto Cloned = CloneModule(Container.getModule(), Map, MustBeCloned);
  return make_unique<DefaultLLVMContainer>(move(Cloned));
}

std::vector<AtomicContract> LLVMEnforcer::getContract() const {
  std::vector<AtomicContract> Contract;
  for (const auto &Element : Passess)
    for (const auto &C : Element->getContract())
      Contract.push_back(C);

  return Contract;
}

void PureLLVMEnforcer::run(DefaultLLVMContainer &Container) {
  legacy::PassManager Manager;
  auto *Registry = PassRegistry::getPassRegistry();
  for (const auto &Element : PassNames) {

    Manager.add(Registry->getPassInfo(Element)->createPass());
  }
  Manager.run(Container.getModule());
}

void LLVMEnforcer::run(DefaultLLVMContainer &Container) {
  legacy::PassManager Manager;
  for (const auto &Element : Passess)
    Element->registerPassess(Manager);
  Manager.run(Container.getModule());
}

LLVMEnforcer &LLVMEnforcer::operator=(const LLVMEnforcer &Other) {
  if (this == &Other)
    return *this;

  llvm::SmallVector<std::unique_ptr<LLVMEnforcerBaseImpl>, 3> NewPassess;

  for (const auto &P : Other.Passess)
    NewPassess.push_back(P->clone());

  Passess = std::move(NewPassess);
  return *this;
}

LLVMEnforcer::LLVMEnforcer(const LLVMEnforcer &Other) {
  for (const auto &P : Other.Passess)
    Passess.push_back(P->clone());
}
