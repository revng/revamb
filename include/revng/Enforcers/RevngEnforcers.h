#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

// WIP: split up

#include <array>

#include "llvm/IR/LegacyPassManager.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/AutoEnforcer/InputOutputContract.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"

namespace AutoEnforcer {

class BinaryContainer;

extern Granularity RootGranularity;
extern Granularity FunctionsGranularity;

extern Kind CFepper;
extern Kind Binary;
extern Kind Root;
extern Kind RootIsolated;
extern Kind Object;
extern Kind Translated;

extern Kind Isolated;
extern Kind ABIEnforced;

extern Kind Dead;

class CFEPEnforcer {
public:
  static constexpr auto Name = "CFepper Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return {
      InputOutputContract(Root, KindExactness::Exact, 0, CFepper, true)
    };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {}
};

#if 0
class LinkEnforcerWithFunctions {
public:
  static constexpr auto Name = "Link Enforcer";
  std::array<InputOutputContract, 3> getContract() const {
    return { InputOutputContract(Support, KindExactness::Exact, 0, Object, 1),
             InputOutputContract(Root, KindExactness::Exact, 0, Dead),
             InputOutputContract(Isolated, KindExactness::Exact, 0, Dead) };
  }
  void run(DefaultLLVMContainer &SourceBinary, BinaryContainer &Binary) {
    revng_abort("Not implemented, i guess here we need a container with "
                "multiple objects files in it?");
  }
};

class O2Enforcer {
public:
  static constexpr auto Name = "02 Enforcer";
  std::array<InputOutputContract, 0> getContract() const { return {}; }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    revng_abort("Not implemented");
  }
};

class LLCRootEnforcer {
public:
  static constexpr auto Name = "LLC Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return {
      InputOutputContract(Root, KindExactness::DerivedFrom, 0, Object, 1)
    };
  }
};

class DetectABIEnforcer {
public:
  static constexpr auto Name = "Detect ABI Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return { InputOutputContract(Root, KindExactness::DerivedFrom, 0) };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    revng_abort("Not implemented");
  }
};

class IsolateEnforcer {
public:
  static constexpr auto Name = "Isolate Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return { InputOutputContract(CFepper, KindExactness::Exact, 0, Isolated) };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    revng_abort("Not implemented");
  }
};

class EnforceABIEnforcer {
public:
  static constexpr auto Name = "EnforceABI Enforcer";
  std::array<InputOutputContract, 1> getContract() const {
    return {
      InputOutputContract(Isolated, KindExactness::Exact, 0, ABIEnforced)
    };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    revng_abort("Not implemented");
  }
};

class InvokeIsolatedEnforcer {
public:
  static constexpr auto Name = "InvokeIsolated Enforcer";
  std::array<InputOutputContract, 2> getContract() const {
    return { InputOutputContract(Root, KindExactness::Exact, 0, RootIsolated),
             InputOutputContract(Isolated, KindExactness::DerivedFrom, 0) };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    revng_abort("Not implemented");
  }
};

#endif

} // namespace AutoEnforcer
