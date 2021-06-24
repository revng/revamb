/// \file SmallMap.cpp
/// \brief Tests for SmallMap

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <functional>

#define BOOST_TEST_MODULE SmallMapLowerBound
bool init_unit_test();
#include "boost/test/unit_test.hpp"

#include "revng/ADT/SmallMap.h"

using namespace llvm;

BOOST_AUTO_TEST_CASE(SmallLowerBound) {
  SmallMap<int, int, 10> Map;
  std::map<int, int> Ref;

  // Elements are inserted in reverse order on purpose, to guard for a bug that
  // is trigger when the internal small-size-optimized container of SmallMap is
  // not sorted.
  Map[4] = 4;
  Map[2] = 2;
  Map[0] = 0;

  Ref[4] = 4;
  Ref[2] = 2;
  Ref[0] = 0;

  auto MIt = Map.lower_bound(3);
  auto RIt = Ref.lower_bound(3);

  revng_check(RIt != Ref.end());
  revng_check(MIt != Map.end());
  revng_check(RIt->second == MIt->second);
}

BOOST_AUTO_TEST_CASE(SmallCustomCompare) {
  SmallMap<int, int, 10, std::greater<int>> Map;
  std::map<int, int, std::greater<int>> Ref;

  // Elements are inserted in reverse order on purpose, to guard for a bug that
  // is trigger when the internal small-size-optimized container of SmallMap is
  // not sorted.
  // Given that we're sorting with std::greater<int> the reverse order is
  // actually the normal order here.
  Map[0] = 0;
  Map[2] = 2;
  Map[4] = 4;

  Ref[0] = 0;
  Ref[2] = 2;
  Ref[4] = 4;

  auto MIt = Map.lower_bound(3);
  auto RIt = Ref.lower_bound(3);

  revng_check(RIt != Ref.end());
  revng_check(MIt != Map.end());
  revng_check(RIt->second == MIt->second);
}

BOOST_AUTO_TEST_CASE(LowerBound) {
  SmallMap<int, int, 1> Map;
  std::map<int, int> Ref;

  Map[4] = 4;
  Map[2] = 2;
  Map[0] = 0;

  Ref[4] = 4;
  Ref[2] = 2;
  Ref[0] = 0;

  auto MIt = Map.lower_bound(3);
  auto RIt = Ref.lower_bound(3);

  revng_check(RIt != Ref.end());
  revng_check(MIt != Map.end());
  revng_check(RIt->second == MIt->second);
}

BOOST_AUTO_TEST_CASE(CustomCompare) {
  SmallMap<int, int, 1, std::greater<int>> Map;
  std::map<int, int, std::greater<int>> Ref;

  Map[0] = 0;
  Map[2] = 2;
  Map[4] = 4;

  Ref[0] = 0;
  Ref[2] = 2;
  Ref[4] = 4;

  auto MIt = Map.lower_bound(3);
  auto RIt = Ref.lower_bound(3);

  revng_check(RIt != Ref.end());
  revng_check(MIt != Map.end());
  revng_check(RIt->second == MIt->second);
}

BOOST_AUTO_TEST_CASE(Contains) {
  SmallMap<int, int, 1, std::greater<int>> Map;
  int Value = 42;

  revng_check(!Map.contains(Value));

  Map.insert({ Value, 1 });
  revng_check(Map.contains(Value));
  revng_check(!Map.contains(Value + 1));
}
