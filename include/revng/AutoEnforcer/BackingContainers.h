#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <fstream>
#include <memory>
#include <set>
#include <sstream>
#include <string>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Error.h"
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
  virtual llvm::Error storeToDisk(llvm::StringRef Path) const = 0;
  virtual llvm::Error loadFromDisk(llvm::StringRef Path) = 0;
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

class StringContainer : public BackingContainer<StringContainer> {
public:
  using TargertContainer = BackingContainersStatus::TargetContainer;
  ~StringContainer() override = default;

  static char ID;

  std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargertContainer &Container) const final {
    auto ToReturn = std::make_unique<StringContainer>();
    for (const auto &Target : Container)
      ToReturn->insert(Target);
    return ToReturn;
  }

  void insert(const AutoEnforcerTarget &Target) {
    ContainedStrings.insert(toString(Target));
  }

  bool contains(const AutoEnforcerTarget &Target) const final {
    return ContainedStrings.count(toString(Target)) != 0;
  }

  void mergeBackDerived(StringContainer &Container) override {
    for (auto &S : Container.ContainedStrings)
      ContainedStrings.insert(S);
  }

  bool remove(const AutoEnforcerTarget &Target) override {
    if (contains(Target))
      return false;

    ContainedStrings.erase(toString(Target));
    return true;
  }

  llvm::Error storeToDisk(llvm::StringRef Path) const override {
    std::error_code EC;
    llvm::raw_fd_ostream OS(Path, EC, llvm::sys::fs::CD_CreateNew);
    if (EC)
      return llvm::createStringError(EC,
                                     "Could not store to file %s",
                                     Path.str().c_str());

    for (const auto &S : ContainedStrings)
      OS << S << "\n";
    return llvm::Error::success();
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) override {
    std::ifstream OS;
    OS.open(Path, std::ios::in | std::ios::trunc);
    if (not OS.is_open())
      return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                     "Could not load file to file %s",
                                     Path.str().c_str());

    std::string S;
    while (getline(OS, S))
      ContainedStrings.insert(S);
    return llvm::Error::success();
  }

private:
  static std::string toString(const AutoEnforcerTarget &Target) {
    std::string ToInsert;
    std::stringstream S(ToInsert);
    AutoEnforcerTarget::dumpQuantifiers(S, Target.getQuantifiers());
    S.flush();
    return ToInsert;
  }

  std::set<std::string> ContainedStrings;
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

  llvm::Error store(llvm::StringRef DirectoryPath) const;
  llvm::Error load(llvm::StringRef DirectoryPath);

  llvm::Expected<const BackingContainerBase *>
  safeGetContainer(llvm::StringRef ContainerName) const;

  llvm::Expected<BackingContainerBase *>
  safeGetContainer(llvm::StringRef ContainerName);

private:
  Map Containers;
};

} // namespace AutoEnforcer
