#pragma once

#include "llvm/ADT/SmallVector.h"

#include "revng/ADT/Iterator.h"
#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/Kind.h"

namespace AutoEnforcer {
class KindsRegisty {
public:
  using Container = llvm::SmallVector<Kind *, 3>;

  KindsRegisty(llvm::SmallVector<Kind *, 3> Kinds = {}) :
    Kinds(std::move(Kinds)) {}
  std::set<AutoEnforcerTarget>
  deduceInvalidations(const InvalidationEventBase &Event);
  void registerKind(Kind &K) { Kinds.push_back(&K); }

  auto begin() { return derefereceIterator(Kinds.begin()); }

  auto end() { return derefereceIterator(Kinds.end()); }

  auto begin() const { return derefereceIterator(Kinds.begin()); }

  auto end() const { return derefereceIterator(Kinds.end()); }

  template<typename OS>
  void dump(OS &OStream) const {
    for (const auto &K : *this)
      OStream << K.getName().str() << "\n";
  }

  void dump() const debug_function { dump(dbg); }

private:
  Container Kinds;
};

} // namespace AutoEnforcer
