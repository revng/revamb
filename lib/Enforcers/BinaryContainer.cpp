//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/Support/Path.h"

#include "revng/Enforcers/BinaryContainer.h"

using namespace llvm;

static void cantFail(std::error_code EC) {
  revng_assert(!EC);
}

namespace AutoEnforcer {

char BinaryContainer::ID;

BinaryContainer::BinaryContainer() {
  cantFail(llvm::sys::fs::createTemporaryFile("", "", Path));
}

BinaryContainer::~BinaryContainer() {
  if (Path.size() > 0)
    cantFail(llvm::sys::fs::remove(Path));
}

std::unique_ptr<BackingContainerBase>
BinaryContainer::cloneFiltered(const TargetContainer &Container) const {
  revng_assert(Container.size() == 1);
  const AutoEnforcerTarget &OnlyTarget = Container[0];
  revng_assert(contains(OnlyTarget));

  auto Result = std::make_unique<BinaryContainer>();
  cantFail(llvm::sys::fs::copy_file(Path, Result->Path));
  return Result;
}

void BinaryContainer::mergeBackDerived(BinaryContainer &&Container) {
  cantFail(llvm::sys::fs::rename(Container.path(), Path));
  Container.Path = "";
}

llvm::Error BinaryContainer::storeToDisk(llvm::StringRef Path) const {
  return errorCodeToError(llvm::sys::fs::copy_file(this->Path, Path));
}

llvm::Error BinaryContainer::loadFromDisk(llvm::StringRef Path) {
  return errorCodeToError(llvm::sys::fs::copy_file(Path, this->Path));
}

} // namespace AutoEnforcer
