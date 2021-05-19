#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <memory>
#include <string>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/MemoryBuffer.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/InputOutputContract.h"

namespace AutoEnforcer {

class BackingContainerBase {
public:
  using TargetContainer = BackingContainersStatus::TargetContainer;
  BackingContainerBase(char const *const ID) : ID(ID) {}
  virtual std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargetContainer &Targets) const = 0;
  virtual void mergeBack(BackingContainerBase &) = 0;
  virtual bool contains(const AutoEnforcerTarget &Target) const = 0;
  virtual bool remove(const AutoEnforcerTarget &Target) = 0;
  virtual ~BackingContainerBase() = default;

  static bool classof(const BackingContainerBase *) { return true; }

  template<typename Derived>
  bool isA() const {
    return ID == &Derived::ID;
  }

  bool remove(llvm::ArrayRef<AutoEnforcerTarget> ToRemove) {
    for (const AutoEnforcerTarget &Target : ToRemove)
      if (auto Erased = remove(Target); not Erased)
        return false;

    return true;
  }

protected:
  char const *const ID;
};

template<typename Derived>
class BackingContainer : public BackingContainerBase {
public:
  BackingContainer() : BackingContainerBase(&Derived::ID) {}

  void mergeBack(BackingContainerBase &Container) final {
    revng_assert(llvm::isa<Derived>(Container));
    mergeBackDerived(llvm::cast<Derived>(Container));
  }
  virtual void mergeBackDerived(Derived &Container) = 0;
  ~BackingContainer() override = default;

  static bool classof(const BackingContainerBase *Base) {
    return Base->isA<Derived>();
  }
};

class BackingContainers {
public:
  using Map = llvm::StringMap<std::unique_ptr<BackingContainerBase>>;
  using const_iterator = Map::const_iterator;
  BackingContainers(Map Containers) : Containers(std::move(Containers)) {}
  BackingContainers() = default;
  BackingContainers(BackingContainers &&) = default;
  BackingContainers &operator=(BackingContainers &&) = default;
  ~BackingContainers() = default;
  BackingContainers(const BackingContainers &) = delete;
  BackingContainers &operator=(const BackingContainers &) = delete;

  BackingContainers cloneFiltered(const BackingContainersStatus &Targets);

  void mergeBackingContainers(BackingContainers &&Other) {
    for (auto &Entry : Other.Containers) {
      revng_assert(Containers.count(Entry.first()) != 0);
      Containers[Entry.first()]->mergeBack(*Entry.second);
    }
  }

  template<typename ContainerDerivedType>
  ContainerDerivedType &get(llvm::StringRef Name) {
    revng_assert(contains(Name));
    auto &Base = *Containers.find(Name)->second;
    revng_assert(llvm::isa<ContainerDerivedType>(Base));
    return llvm::cast<ContainerDerivedType>(Base);
  }

  template<typename ContainerDerivedType>
  const ContainerDerivedType &get(llvm::StringRef Name) const {
    revng_assert(contains(Name));
    const auto &Base = *Containers.find(Name)->second;
    revng_assert(llvm::isa<ContainerDerivedType>(Base));
    return llvm::cast<ContainerDerivedType>(Base);
  }

  BackingContainerBase &get(llvm::StringRef Name) {
    revng_assert(contains(Name));
    return *Containers.find(Name)->second;
  }

  const BackingContainerBase &get(llvm::StringRef Name) const {

    revng_assert(contains(Name));
    return *Containers.find(Name)->second;
  }

  bool contains(llvm::StringRef Name) const {
    return Containers.count(Name) != 0;
  }

  bool contains(const AutoEnforcerTarget &Target) const;

  void add(std::string Name, std::unique_ptr<BackingContainerBase> Container) {
    Containers.insert_or_assign(Name, (std::move(Container)));
  }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    for (const auto &Entry : Containers) {
      indent(OS, Indents);
      OS << Entry.first().str() << "\n";
    }
  }

  void dump() const { dump(dbg); }

  const_iterator begin() const { return Containers.begin(); }
  const_iterator end() const { return Containers.end(); }

  llvm::Error remove(const BackingContainersStatus &ToRemove);
  void intersect(BackingContainersStatus &ToIntersect) const;

private:
  Map Containers;
};

} // namespace AutoEnforcer
