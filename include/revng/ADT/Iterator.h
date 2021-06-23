#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <iterator>
#include <type_traits>

#include "llvm/ADT/STLExtras.h"

auto derefereceIterator(auto Iter) {
  return llvm::map_iterator(Iter, [](const auto &Ptr) -> decltype(*Ptr) & {
    return *Ptr;
  });
}

auto derefereceRange(auto &&Range) {
  return llvm::make_range(derefereceIterator(Range.begin()),
                          derefereceIterator(Range.end()));
}
