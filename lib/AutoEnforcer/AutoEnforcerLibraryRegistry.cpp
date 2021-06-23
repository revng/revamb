#include "revng/AutoEnforcer/AutoEnforcerLibraryRegistry.h"

using namespace AutoEnforcer;

llvm::SmallVector<AutoEnforcerLibraryRegistry *, 3> &
AutoEnforcerLibraryRegistry::getInstances() {
  static llvm::SmallVector<AutoEnforcerLibraryRegistry *, 3> ToReturn;
  return ToReturn;
}
