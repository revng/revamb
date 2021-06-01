#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

// WIP: reduce headers
#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/AutoEnforcer/InputOutputContract.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/Enforcers/RevngEnforcers.h"

namespace AutoEnforcer {

class LinkSupportEnforcer {
public:
  static constexpr auto Name = "LLVM Link Support Enforcer";

  std::array<AtomicContract, 0> getContract() const { return {}; }

  void run(DefaultLLVMContainer &TargetContainer);
};

} // namespace AutoEnforcer
