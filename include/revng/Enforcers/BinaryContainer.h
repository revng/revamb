#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/Support/Assert.h"

namespace Model {
class BinaryContainer : public BackingContainer<BinaryContainer> {
public:
  BinaryContainer() = default;
  ~BinaryContainer() override = default;
  using TargetContainer = BackingContainersStatus::TargetContainer;

  std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargetContainer &Container) const final {
    revng_abort("Not implemented!!! you have to tell me how to clone the "
                "particular "
                "targets, i guess in case of a binary means clone us all");
    return nullptr;
  }

  bool contains(const AutoEnforcerTarget &Target) const final {
    revng_abort("Not implemented!!! you have to tell me how check if a "
                "particular target exists");
    return false;
  }

  void mergeBackDerived(BinaryContainer &Container) override {
    revng_abort("You have to tell me how to merge back the a binary "
                "container into this one, probably a binary can only be "
                "used as a source and this should never be used?");
  }

  static char ID;

private:
};

} // namespace Model
