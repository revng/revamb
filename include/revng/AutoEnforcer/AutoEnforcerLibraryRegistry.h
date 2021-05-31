#pragma once

#include "llvm/ADT/SmallVector.h"

#include "revng/AutoEnforcer/PipelineLoader.h"

namespace AutoEnforcer {

class AutoEnforcerLibraryRegistry {
public:
  AutoEnforcerLibraryRegistry() { getInstances().push_back(this); }

  static void registerAllContainersAndEnforcers(PipelineLoader &Loader) {
    for (const auto &Reg : getInstances())
      Reg->registerContainersAndEnforcers(Loader);
  }

  static void registerAllKinds(llvm::StringMap<Kind *> &KindDictionary) {
    for (const auto &Reg : getInstances())
      Reg->registerKinds(KindDictionary);

    for (auto &Kind : KindDictionary) {
      Kind.second->getRootAncestor()->assign();
      Kind.second->Granularity->getRootAncestor()->assign();
    }
  }

  virtual ~AutoEnforcerLibraryRegistry(){};

  virtual void registerContainersAndEnforcers(PipelineLoader &Loader) = 0;
  virtual void registerKinds(llvm::StringMap<Kind *> &KindDictionary) = 0;

private:
  static llvm::SmallVector<AutoEnforcerLibraryRegistry *, 3> &getInstances();
};

} // namespace AutoEnforcer
