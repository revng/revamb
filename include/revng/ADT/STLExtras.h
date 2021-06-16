#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <type_traits>

#include "llvm/ADT/STLExtras.h"

//
// is_integral
//
template<typename T>
concept Integral = std::is_integral_v<T>;

//
// is_specialization
//
template<typename Test, template<typename...> class Ref>
struct is_specialization : std::false_type {};

template<template<typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref> : std::true_type {};

template<typename Test, template<typename...> class Ref>
constexpr bool is_specialization_v = is_specialization<Test, Ref>::value;

static_assert(is_specialization_v<std::vector<int>, std::vector>);
static_assert(is_specialization_v<std::pair<int, long>, std::pair>);

//
// HasTupleSize
//

template<typename T>
concept HasTupleSize = requires {
  std::tuple_size<T>::value;
};

// clang-format off
template<typename T, std::size_t I>
concept IsTupleEnd
  = std::is_same_v<std::true_type,
                   std::integral_constant<bool, std::tuple_size_v<T> == I>>;

template<typename T, std::size_t I>
concept IsNotTupleEnd
  = std::is_same_v<std::true_type,
                   std::integral_constant<bool, std::tuple_size_v<T> != I>>;
// clang-format on

static_assert(HasTupleSize<std::tuple<>>);
static_assert(!HasTupleSize<std::vector<int>>);
static_assert(!HasTupleSize<int>);

//===----------------------------------------------------------------------===//
//     Extra additions to <iterator>
//===----------------------------------------------------------------------===//

namespace revng {
namespace detail {
template<typename ItTy,
         typename FuncTy,
         typename FuncReturnTy = decltype(
           std::declval<FuncTy>()(*std::declval<ItTy>()))>
class proxy_mapped_iterator_impl : public llvm::mapped_iterator<ItTy, FuncTy> {
  struct IteratorProxy {
    IteratorProxy(FuncReturnTy &&Value) : Temporary(std::move(Value)) {}
    FuncReturnTy *const operator->() { return &Temporary; }
    FuncReturnTy const *const operator->() const { return &Temporary; }

  private:
    FuncReturnTy Temporary;
  };

public:
  using llvm::mapped_iterator<ItTy, FuncTy>::mapped_iterator;
  using reference = std::decay_t<FuncReturnTy>;

  IteratorProxy operator->() {
    return llvm::mapped_iterator<ItTy, FuncTy>::operator*();
  }
  IteratorProxy const operator->() const {
    return llvm::mapped_iterator<ItTy, FuncTy>::operator*();
  }
};
} // namespace detail

// `mapped_iterator` - this is a specialized version of `llvm::mapped_iterator`
// with improved temporary object support.
template<typename ItTy, typename FuncTy>
using mapped_iterator = std::conditional_t<
  std::is_object_v<decltype(std::declval<FuncTy>()(*std::declval<ItTy>()))>,
  detail::proxy_mapped_iterator_impl<ItTy, FuncTy>,
  llvm::mapped_iterator<ItTy, FuncTy>>;

// `map_iterator` - Provide a convenient way to create `mapped_iterator`s,
// just like `make_pair` is useful for creating pairs...
template<class ItTy, class FuncTy>
inline auto map_iterator(ItTy I, FuncTy F) {
  return mapped_iterator<ItTy, FuncTy>(std::move(I), std::move(F));
};

template<class ContainerTy, class FuncTy>
auto map_range(ContainerTy &&C, FuncTy F) {
  return llvm::make_range(map_iterator(C.begin(), F), map_iterator(C.end(), F));
}
} // namespace revng
