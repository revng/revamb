#pragma once

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/InputOutputContract.h"

namespace AutoEnforcer {

template<typename Source, typename Destination = Source>
class CopyEnforcer {
public:
  CopyEnforcer(Kind &K) : K(&K) {}

  static constexpr auto Name = "Copy Enforcer";
  std::array<AtomicContract, 1> getContract() const {
    return { AtomicContract(*K, KindExactness::Exact, 0, *K, 1) };
  }

  void run(const Source &S, Destination &T) { T = S; }

private:
  Kind *K;
};

} // namespace AutoEnforcer
