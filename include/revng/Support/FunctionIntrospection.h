#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <functional>
#include <tuple>

#include "revng/Support/TupleIntrospection.h"

template<class T>
struct function_traits : function_traits<decltype(&T::operator())> {};

// partial specialization for function type
template<class R, class... Args>
struct function_traits<R(Args...)> {
  using result_type = R;
  using argument_types = std::tuple<Args...>;
};

// partial specialization for function pointer
template<class R, class... Args>
struct function_traits<R (*)(Args...)> {
  using result_type = R;
  using argument_types = std::tuple<Args...>;
};

// partial specialization for std::function
template<class R, class... Args>
struct function_traits<std::function<R(Args...)>> {
  using result_type = R;
  using argument_types = std::tuple<Args...>;
};

// partial specialization for pointer-to-member-function (i.e., operator()'s)
template<class T, class R, class... Args>
struct function_traits<R (T::*)(Args...)> {
  using result_type = R;
  using argument_types = std::tuple<Args...>;
};

template<class T, class R, class... Args>
struct function_traits<R (T::*)(Args...) const> {
  using result_type = R;
  using argument_types = std::tuple<Args...>;
};

template<typename T>
using function_args_t = typename function_traits<T>::argument_types;

template<size_t Index, typename T>
using function_arg_t = typename std::tuple_element_t<Index, function_args_t<T>>;

template<size_t Index, typename T>
using decayed_function_arg_t = typename std::decay_t<function_arg_t<Index, T>>;

template<typename T>
using function_ret_t = typename function_traits<T>::result_type;

template<typename T>
using function_args_tuple = typename function_traits<T>::argument_types;
