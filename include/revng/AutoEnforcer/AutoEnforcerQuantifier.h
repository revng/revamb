#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <string>
#include <optional>
#include <utility>


#include "llvm/ADT/SmallVector.h"
#include "revng/Support/Assert.h"
#include "revng/Support/Debug.h"

namespace AutoEnforcer {
class AutoEnforcerQuantifier {
public:
  AutoEnforcerQuantifier(std::optional<std::string> Name) :
    Name(std::move(Name)) {}

  AutoEnforcerQuantifier() : Name(std::nullopt) {}

  bool isAll() const { return not Name.has_value(); }
  bool isSingle() const { return Name.has_value(); }

  const std::string &getName() const {
    revng_assert(isSingle());
    return *Name;
  }

  bool operator<(const AutoEnforcerQuantifier &Other) const {
    return Name < Other.Name;
  }

  template<typename OStream>
  void dump(OStream &OS) const debug_function {
    if (Name.has_value())
      OS << *Name;
    else
      OS << "*";
  }

  void dump() const debug_function { dump(dbg); }

  bool operator<=>(const AutoEnforcerQuantifier &Other) const {
    if (not Name.has_value() and not Other.Name.has_value())
      return 0;
    if (not Name.has_value())
      return -1;
    if (not Other.Name.has_value())
      return 1;
    return strcmp(Name->c_str(), Other.Name->c_str());
  }

private:
  std::optional<std::string> Name;
};

using GranularityList = llvm::SmallVector<AutoEnforcerQuantifier, 3>;

}
