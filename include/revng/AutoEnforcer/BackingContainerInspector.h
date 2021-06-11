#pragma once

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/Support/ManagedStatic.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainers.h"

namespace AutoEnforcer {

template<typename BackingContainer>
class BackingContainerInspector {
public:
  using StaticContainer = llvm::SmallVector<BackingContainerInspector *, 4>;
  BackingContainerInspector(Kind &K) : K(&K) {
    getRegisteredInspectors().push_back(this);
  }
  virtual ~BackingContainerInspector() = default;

  virtual bool contains(const AutoEnforcerTarget &Target,
                        const BackingContainer &Container) const = 0;
  virtual bool remove(const AutoEnforcerTarget &Target,
                      BackingContainer &Container) const = 0;

  static bool containsTarget(const AutoEnforcerTarget &Target,
                             const BackingContainer &Container) {
    for (const BackingContainerInspector *Inspector :
         getRegisteredInspectors()) {
      if (Inspector->K != &Target.getKind())
        continue;

      return Inspector->contains(Target, Container);
    }
    return false;
  }

  static bool
  removeTarget(const AutoEnforcerTarget &Target, BackingContainer &Container) {
    for (const BackingContainerInspector *Inspector :
         getRegisteredInspectors()) {
      if (Inspector->K != &Target.getKind())
        continue;

      return Inspector->remove(Target, Container);
    }
    return false;
  }

private:
  Kind *K;
  static StaticContainer &getRegisteredInspectors() {
    static StaticContainer Container;
    return Container;
  }
};

template<typename Derived>
class InspectableBackingContainer : public BackingContainer<Derived> {
public:
  InspectableBackingContainer() : BackingContainer<Derived>() {}
  ~InspectableBackingContainer() override = default;

  bool contains(const AutoEnforcerTarget &Target) const final {
    return BackingContainerInspector<Derived>::containsTarget(Target, *self());
  }

  bool remove(const AutoEnforcerTarget &Target) final {
    return BackingContainerInspector<Derived>::removeTarget(Target, *self());
  }

  Derived *self() { return static_cast<Derived *>(this); }

  const Derived *self() const { return static_cast<const Derived *>(this); }
};
} // namespace AutoEnforcer
