//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"

using namespace AutoEnforcer;
using namespace std;
using namespace llvm;

void BackingContainersStatus::removeDupplicates() {
  for (auto &Pair : ContainersStatus) {
    auto &Container = Pair.second;
    sort(Container);
    auto Last = unique(Container.begin(), Container.end());
    Container.erase(Last, Container.end());
  }
}

int AutoEnforcerTarget::operator<=>(const AutoEnforcerTarget &Other) const {
  if (K > Other.K)
    return -1;
  if (K < Other.K)
    return 1;

  if (Exact > Other.Exact)
    return -1;
  if (Exact < Other.Exact)
    return 1;

  if (Entries.size() != Other.Entries.size()) {
    if (Entries.size() > Other.Entries.size())
      return -1;
    if (Entries.size() < Other.Entries.size())
      return 1;
  }

  for (const auto &[l, r] : zip(Entries, Other.Entries)) {
    auto Val = l <=> r;
    if (Val != 0)
      return Val;
  }

  return 0;
}
