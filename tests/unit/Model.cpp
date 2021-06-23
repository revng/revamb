/// \file Model.cpp
/// \brief

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#define BOOST_TEST_MODULE Model
bool init_unit_test();
#include "boost/test/unit_test.hpp"

#include "revng/Model/Binary.h"
#include "revng/Model/TupleTreeDiff.h"

using namespace model;

auto ARM1000 = MetaAddress::fromString("0x1000:Code_arm");
auto ARM2000 = MetaAddress::fromString("0x2000:Code_arm");
auto ARM3000 = MetaAddress::fromString("0x3000:Code_arm");

BOOST_AUTO_TEST_CASE(TestIntrospection) {
  using namespace llvm;

  Function TheFunction(MetaAddress::invalid());

  // Use get
  TheFunction.Name = "FunctionName";
  revng_check(get<1>(TheFunction) == "FunctionName");

  // Test std::tuple_size
  static_assert(std::tuple_size<Function>::value >= 2);

  // Test TupleLikeTraits
  using TLT = TupleLikeTraits<Function>;
  static_assert(std::is_same_v<std::tuple_element_t<1, Function>,
                               decltype(TheFunction.Name)>);
  revng_check(StringRef(TLT::name()) == "model::Function");
  revng_check(StringRef(TLT::fieldName<1>()) == "Name");
}

BOOST_AUTO_TEST_CASE(TestPathAccess) {
  Binary TheBinary;
  TupleTreePath Zero;
  Zero.push_back(size_t(0));
  auto *FirstField = getByPath<std::string>(Zero, TheBinary);
  revng_check(FirstField == &TheBinary.Name);

  using FunctionsType = decltype(TheBinary.Functions);
  auto *FunctionsField = getByPath<FunctionsType>("/Functions", TheBinary);
  revng_check(FunctionsField == &TheBinary.Functions);

  // Test non existing field
  revng_check(getByPath<FunctionsType>("/Function", TheBinary) == nullptr);

  // Test non existing entry in container
  revng_check(getByPath<Function>("/Functions/:Invalid", TheBinary) == nullptr);

  // Test existing entry in container
  Function &F = TheBinary.Functions[MetaAddress::invalid()];
  revng_check(getByPath<Function>("/Functions/:Invalid", TheBinary) == &F);
}

template<>
struct llvm::yaml::ScalarTraits<std::pair<MetaAddress, MetaAddress>>
  : CompositeScalar<std::pair<MetaAddress, MetaAddress>, '-'> {};

BOOST_AUTO_TEST_CASE(TestCompositeScalar) {
  // MetaAddress pair
  {
    using BlockKeyPair = std::pair<MetaAddress, MetaAddress>;
    BlockKeyPair BlockKey = { ARM2000, ARM3000 };
    auto BlockKeyName = getNameFromYAMLScalar(BlockKey);
    revng_check(BlockKeyName == "0x2000:Code_arm-0x3000:Code_arm");
  }
}

BOOST_AUTO_TEST_CASE(TestStringPathConversion) {
  revng_check(stringAsPath<Binary>("/").value() == TupleTreePath{});
  TupleTreePath Zero;
  Zero.push_back(size_t(0));
  revng_check(stringAsPath<Binary>("/Name").value() == Zero);

  TupleTreePath InvalidFunctionPath;
  InvalidFunctionPath.push_back(size_t(3));
  InvalidFunctionPath.push_back(MetaAddress::invalid());
  auto MaybeInvalidFunctionPath = stringAsPath<Binary>("/Functions/:Invalid");
  revng_check(MaybeInvalidFunctionPath.value() == InvalidFunctionPath);

  TupleTreePath InvalidFunctionNamePath = InvalidFunctionPath;
  InvalidFunctionNamePath.push_back(size_t(1));
  auto MaybePath = stringAsPath<Binary>("/Functions/:Invalid/Name");
  revng_check(MaybePath.value() == InvalidFunctionNamePath);

  auto CheckRoundTrip = [](const char *String) {
    auto Path = stringAsPath<Binary>(String).value();
    auto StringAgain = pathAsString<Binary>(Path);
    revng_check(StringAgain == String);
  };

  CheckRoundTrip("/Functions");
  CheckRoundTrip("/Functions/:Invalid");
  CheckRoundTrip("/Functions/:Invalid/Entry");
  CheckRoundTrip("/Functions/0x1000:Code_arm/Entry");
  CheckRoundTrip("/Functions/0x1000:Code_arm/CFG/0x2000:Code_arm/Start");
  CheckRoundTrip("/Functions/0x1000:Code_arm/CFG/0x2000:Code_arm/Successors"
                 "/0x2000:Code_arm-DirectBranch/Destination");
}

BOOST_AUTO_TEST_CASE(TestPathMatcher) {
  //
  // Single matcher
  //
  {
    auto Matcher = PathMatcher::create<Binary>("/Functions/*/Entry").value();

    auto ARM1000EntryPath = pathAsString<Binary>(Matcher.apply(ARM1000));
    revng_check(ARM1000EntryPath == "/Functions/0x1000:Code_arm/Entry");

    auto MaybeToMatch = stringAsPath<Binary>("/Functions/0x1000:Code_arm/"
                                             "Entry");
    auto MaybeMatch = Matcher.match<MetaAddress>(MaybeToMatch.value());
    revng_check(MaybeMatch);
    revng_check(std::get<0>(*MaybeMatch) == ARM1000);
  }

  //
  // Double matcher
  //
  {
    auto MaybeMatcher = PathMatcher::create<Binary>("/Functions/*/CFG/*/Start");
    auto Matcher = MaybeMatcher.value();

    auto ARM1000EntryPath = Matcher.apply(ARM1000, ARM2000);
    auto ARM1000EntryPathAsString = pathAsString<Binary>(ARM1000EntryPath);
    auto ExpectedName = ("/Functions/0x1000:Code_arm/CFG/"
                         "0x2000:Code_arm/Start");
    revng_check(ARM1000EntryPathAsString == ExpectedName);

    auto Match = Matcher.match<MetaAddress, MetaAddress>(ARM1000EntryPath);
    revng_check(Match);
    revng_check(std::get<0>(*Match) == ARM1000);
    revng_check(std::get<1>(*Match) == ARM2000);

    {
      auto Path = stringAsPath<Binary>("/Functions");
      revng_check((not Matcher.match<MetaAddress, MetaAddress>(Path.value())));
    }

    {
      auto Path = stringAsPath<Binary>("/Functions/:Invalid");
      revng_check((not Matcher.match<MetaAddress, MetaAddress>(Path.value())));
    }
  }
}

namespace TestTupleTree {
class Element;
class Root;
} // namespace TestTupleTree

class TestTupleTree::Element {
public:
  int Key;
  TupleTreeReference<TestTupleTree::Element, TestTupleTree::Root> Self;
};
INTROSPECTION_NS(TestTupleTree, Element, Key, Self)

template<>
struct KeyedObjectTraits<TestTupleTree::Element> {
  static int key(const TestTupleTree::Element &Obj) { return Obj.Key; }

  static TestTupleTree::Element fromKey(const int &Key) {
    return TestTupleTree::Element{ Key, {} };
  }
};

class TestTupleTree::Root {
public:
  SortedVector<TestTupleTree::Element> Elements;
};

INTROSPECTION_NS(TestTupleTree, Root, Elements)

BOOST_AUTO_TEST_CASE(TestTupleTreeReference) {
  using namespace TestTupleTree;

  using Reference = TupleTreeReference<TestTupleTree::Element,
                                       TestTupleTree::Root>;

  TupleTree<Root> TheRoot;
  Element &AnElement = TheRoot->Elements[3];
  AnElement.Self = Reference::fromString("/Elements/3");

  TheRoot.initializeReferences();

  revng_check(AnElement.Self.get() == &AnElement);
}

BOOST_AUTO_TEST_CASE(TestTupleTreeDiff) {
  if (false) {
    model::Binary Left;
    model::Binary Right;
    diff(Left, Right).dump();
  }
}

static_assert(std::is_default_constructible_v<TupleTree<TestTupleTree::Root>>);
static_assert(not std::is_copy_assignable_v<TupleTree<TestTupleTree::Root>>);
static_assert(not std::is_copy_constructible_v<TupleTree<TestTupleTree::Root>>);
static_assert(std::is_move_assignable_v<TupleTree<TestTupleTree::Root>>);
static_assert(std::is_move_constructible_v<TupleTree<TestTupleTree::Root>>);
