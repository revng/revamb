#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//
//
#include <set>

#include "revng/ADT/Hierarchy.h"
#include "revng/AutoEnforcer/AutoEnforcerQuantifier.h"
#include "revng/AutoEnforcer/InvalidationEvent.h"
#include "revng/Support/Assert.h"

namespace AutoEnforcer {

struct Granularity : public HierarchyNode<Granularity> {
public:
  Granularity(llvm::StringRef Name) : HierarchyNode(Name) {}
  Granularity(llvm::StringRef Name, Granularity &Parent) :
    HierarchyNode<Granularity>(Name, Parent) {}
};

struct Kind : public HierarchyNode<Kind> {
public:
  Kind(llvm::StringRef Name, Granularity *Granularity) :
    HierarchyNode<Kind>(Name), Granularity(Granularity) {
    revng_assert(Granularity != nullptr);
  }
  Kind(llvm::StringRef Name, Kind &Parent, Granularity *Granularity) :
    HierarchyNode<Kind>(Name, Parent), Granularity(Granularity) {
    revng_assert(Granularity != nullptr);
  }

  virtual void deduceInvalidations(const InvalidationEventBase &,
                                   std::set<GranularityList> &Targets) const {
    GranularityList List;
    for (size_t i = 0; i < Granularity->depth(); i++)
      List.push_back(AutoEnforcerQuantifier());
    Targets.insert(std::move(List));
  }

  Granularity *Granularity;

  virtual ~Kind() = default;
};

} // namespace AutoEnforcer
