#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//
#include <tuple>

template<typename T, typename Tuple>
struct has_type;

template<typename T, typename... Us>
struct has_type<T, std::tuple<Us...>>
  : std::disjunction<std::is_same<T, Us>...> {};

template<size_t I, typename T, typename Tuple_t>
constexpr size_t index_in_tuple_fn() {
  static_assert(I < std::tuple_size<Tuple_t>::value,
                "The element is not in the tuple");

  typedef typename std::tuple_element<I, Tuple_t>::type el;
  if constexpr (std::is_same<T, el>::value) {
    return I;
  } else {
    return index_in_tuple_fn<I + 1, T, Tuple_t>();
  }
}

template<typename T, typename Tuple_t>
struct index_in_tuple {
  static constexpr size_t value = index_in_tuple_fn<0, T, Tuple_t>();
};

template<typename T, typename Tuple_t>
constexpr size_t index_in_tuple_v() {
  return index_in_tuple<T, Tuple_t>::value;
}
