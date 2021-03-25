#include "revng/AutoEnforcer/AutoEnforcer.h"

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

using namespace std;
using namespace llvm;
using namespace Model;

using InvalidationMap = StringMap<BackingContainersStatus>;
Expected<InvalidationMap>
AutoEnforcer::getInvalidations(const AutoEnforcerTarget &Target) const {
  llvm::StringMap<BackingContainersStatus> Invalidations;
  for (const auto &Step : Pipeline)
    for (const auto &Container : Step.getBackingContainers())
      if (Container.second->contains(Target))
        Invalidations[Step.getName()].add(Container.first(), Target);
  if (auto Error = deduceInvalidations(Invalidations); Error)
    return move(Error);

  return move(Invalidations);
}

Error AutoEnforcer::invalidate(const AutoEnforcerTarget &Target) {
  auto Invalidations = getInvalidations(Target);
  if (not Invalidations)
    return Invalidations.takeError();
  return Pipeline.invalidate(*Invalidations);
}
