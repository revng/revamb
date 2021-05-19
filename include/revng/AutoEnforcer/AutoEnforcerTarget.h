#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <initializer_list>
#include <iterator>
#include <optional>
#include <utility>
#include <vector>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/ADT/Hierarchy.h"
#include "revng/Support/Assert.h"
#include "revng/Support/Debug.h"

namespace AutoEnforcer {
namespace detail {
struct EmptySturct {};
}; // namespace detail

using Granularity = HierarchyNode<detail::EmptySturct>;
using Kind = HierarchyNode<Granularity *>;

enum class KindExactness { Exact, DerivedFrom };

class AutoEnforcerQuantifier {
public:
  AutoEnforcerQuantifier(std::optional<std::string> Name) :
    Name(std::move(Name)) {}

  AutoEnforcerQuantifier() : Name(std::nullopt) {}

  bool isAll() const { return not Name.has_value(); }
  bool isSingle() const { return Name.has_value(); }

  const std::string &getName() const {
    revng_assert(isSingle());
    return *Name;
  }

  bool operator<(const AutoEnforcerQuantifier &Other) const {
    return Name < Other.Name;
  }

  template<typename OStream>
  void dump(OStream &OS) const {
    if (Name.has_value())
      OS << *Name;
    else
      OS << "*";
  }

  void dump() const { dump(dbg); }

private:
  std::optional<std::string> Name;
};

class AutoEnforcerTarget {
public:
  AutoEnforcerTarget(llvm::SmallVector<AutoEnforcerQuantifier, 3> Quantifiers,
                     const Kind &K,
                     KindExactness Exactness = KindExactness::Exact) :
    Entries(std::move(Quantifiers)), K(&K), Exact(Exactness) {}

  AutoEnforcerTarget(AutoEnforcerQuantifier Quantifier,
                     const Kind &K,
                     KindExactness Exactness = KindExactness::Exact) :
    Entries({ std::move(Quantifier) }), K(&K), Exact(Exactness) {}

  AutoEnforcerTarget(std::string Name,
                     const Kind &K,
                     KindExactness Exactness = KindExactness::Exact) :
    Entries({ AutoEnforcerQuantifier(std::move(Name)) }),
    K(&K),
    Exact(Exactness) {}

  AutoEnforcerTarget(std::initializer_list<std::string> Names,
                     const Kind &K,
                     KindExactness Exactness = KindExactness::Exact) :
    K(&K), Exact(Exactness) {
    for (auto Name : Names)
      Entries.emplace_back(std::move(Name));
  }

  bool operator<(const AutoEnforcerTarget &Other) const {
    auto Self = std::tie(Entries, K, Exact);
    auto OtherSelf = std::tie(Other.Entries, Other.K, Other.Exact);
    return Self < OtherSelf;
  }

  const Kind &getKind() const { return *K; }

  KindExactness kindExactness() const { return Exact; }
  void setKind(const Kind &NewKind) { K = &NewKind; }
  void setKindExactness(KindExactness NewExactness) { Exact = NewExactness; }

  const llvm::SmallVector<AutoEnforcerQuantifier, 3> &getQuantifiers() const {
    return Entries;
  }

  void addGranularity() { Entries.emplace_back(); }
  void dropGranularity() { Entries.pop_back(); }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    indent(OS, Indents);

    OS << (Exact == KindExactness::DerivedFrom ? "derived from " : "exactly ")
       << K->getName().str() << " With path: ";
    for (const auto &Entry : Entries) {
      Entry.dump(OS);
      OS << "/";
    }
    OS << "\n";
  }

  void dump() const debug_function { dump(dbg); }

private:
  llvm::SmallVector<AutoEnforcerQuantifier, 3> Entries;
  const Kind *K;
  KindExactness Exact;
};

class BackingContainersStatus {
public:
  using TargetContainer = llvm::SmallVector<AutoEnforcerTarget, 3>;

  TargetContainer &operator[](llvm::StringRef ContainerName) {
    return ContainersStatus[ContainerName];
  }

  TargetContainer &at(llvm::StringRef ContainerName) {
    return ContainersStatus.find(ContainerName)->getValue();
  }

  const TargetContainer &at(llvm::StringRef ContainerName) const {
    return ContainersStatus.find(ContainerName)->getValue();
  }

  auto begin() { return ContainersStatus.begin(); }
  auto end() { return ContainersStatus.end(); }
  auto begin() const { return ContainersStatus.begin(); }
  auto end() const { return ContainersStatus.end(); }

  bool empty() const { return size() == 0; }
  size_t size() const {
    size_t Size = 0;
    for (const auto &Container : ContainersStatus)
      Size += Container.second.size();
    return Size;
  }

  void add(llvm::StringRef Name,
           std::initializer_list<std::string> Names,
           const Kind &K,
           KindExactness Exactness = KindExactness::Exact) {
    ContainersStatus[Name].emplace_back(Names, K, Exactness);
  }

  void add(llvm::StringRef Name, AutoEnforcerTarget Target) {
    ContainersStatus[Name].emplace_back(std::move(Target));
  }

  bool contains(llvm::StringRef ContainerName) const {
    return ContainersStatus.find(ContainerName) != ContainersStatus.end();
  }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    indent(OS, Indents);
    OS << "{\n";
    for (const auto &Container : ContainersStatus) {
      indent(OS, Indents + 1);
      OS << Container.first().str() << ":\n";
      for (const auto &Entry : Container.second)
        Entry.dump(OS, Indents + 2);
    }
    indent(OS, Indents);
    OS << "}\n";
  }

  void dump() const { dump(dbg); }

  void merge(const BackingContainersStatus &Other);

private:
  llvm::StringMap<TargetContainer> ContainersStatus;
};

} // namespace AutoEnforcer
