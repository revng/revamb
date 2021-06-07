#pragma once

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"

#include "revng/AutoEnforcer/PipelineLoader.h"

namespace AutoEnforcer {

class AutoEnforcerLibraryRegistry {
public:
  AutoEnforcerLibraryRegistry() { getInstances().push_back(this); }

  static void registerAllContainersAndEnforcers(PipelineLoader &Loader) {
    for (const auto &Reg : getInstances())
      Reg->registerContainersAndEnforcers(Loader);
  }

  static KindsRegisty registerAllKinds() {
    KindsRegisty Registry;
    for (const auto &Reg : getInstances())
      Reg->registerKinds(Registry);

    for (auto &Kind : Registry) {
      Kind.getRootAncestor()->assign();
      Kind.Granularity->getRootAncestor()->assign();
    }
    return Registry;
  }

  virtual ~AutoEnforcerLibraryRegistry(){};

  virtual void registerContainersAndEnforcers(PipelineLoader &Loader) = 0;
  virtual void registerKinds(KindsRegisty &KindDictionary) = 0;

private:
  static llvm::SmallVector<AutoEnforcerLibraryRegistry *, 3> &getInstances();
};
} // namespace AutoEnforcer
