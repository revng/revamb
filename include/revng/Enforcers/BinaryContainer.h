#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/Support/Assert.h"

namespace AutoEnforcer {

// WIP: rename to temporary file
class BinaryContainer : public BackingContainer<BinaryContainer> {
private:
  llvm::SmallString<32> Path;

public:
  BinaryContainer();
  BinaryContainer(BinaryContainer &&);
  ~BinaryContainer() override;
  BinaryContainer(const BinaryContainer &) = delete;

  using TargetContainer = BackingContainersStatus::TargetContainer;

  std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargetContainer &Container) const final;

  bool contains(const AutoEnforcerTarget &Target) const final {
    return not Path.empty();
  }

  bool remove(const AutoEnforcerTarget &Target) final { revng_abort(); }

  void mergeBackDerived(BinaryContainer &&Container) override;

  llvm::Error storeToDisk(llvm::StringRef Path) const override;

  llvm::Error loadFromDisk(llvm::StringRef Path) override;

  static char ID;

public:
  llvm::StringRef path() const { return llvm::StringRef(Path); }

  void dump() const debug_function { dbg << Path.data() << "\n"; }
};

} // namespace AutoEnforcer
