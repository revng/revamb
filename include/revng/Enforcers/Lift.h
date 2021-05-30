#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <array>

#include "llvm/IR/LegacyPassManager.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/AutoEnforcer/InputOutputContract.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/Enforcers/RevngEnforcers.h"

extern llvm::cl::opt<unsigned long long> BaseAddress;

namespace AutoEnforcer {

class LiftEnforcer {
public:
  static constexpr auto Name = "Lift Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return {
      InputOutputContract(Binary, KindExactness::Exact, 0, Root, 1, true)
    };
  }

  void run(const BinaryContainer &SourceBinary,
           DefaultLLVMContainer &TargetContainer);
};

} // namespace AutoEnforcer
