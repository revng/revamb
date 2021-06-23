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

class LinkForTranslationEnforcer {
public:
  static constexpr auto Name = "Link for Translation Enforcer";

  std::array<AtomicContract, 1> getContract() const {
    return { AtomicContract({ InputOutputContract(Root,
                                                  KindExactness::DerivedFrom,
                                                  0,
                                                  Translated,
                                                  3,
                                                  true),
                              InputOutputContract(Binary,
                                                  KindExactness::Exact,
                                                  1,
                                                  Translated,
                                                  3,
                                                  true),
                              InputOutputContract(Object,
                                                  KindExactness::Exact,
                                                  2,
                                                  Translated,
                                                  3,
                                                  true) }) };
  }

  void run(DefaultLLVMContainer &M,
           BinaryContainer &InputBinary,
           BinaryContainer &ObjectFile,
           BinaryContainer &OutputBinary);
};

} // namespace AutoEnforcer
