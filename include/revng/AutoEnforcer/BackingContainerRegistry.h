#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <memory>
#include <string>
#include <utility>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"

#include "revng/AutoEnforcer/BackingContainers.h"

namespace Model {
class BackingContainerRegistryEntry {
public:
  virtual std::unique_ptr<BackingContainerBase> createEmpty() const = 0;
  virtual ~BackingContainerRegistryEntry() = default;
};

class BackingContainerFunctionFactoryWrapper
  : public BackingContainerRegistryEntry {
public:
  using CreatorFunctionType = std::unique_ptr<BackingContainerBase> (*)(void);
  BackingContainerFunctionFactoryWrapper(CreatorFunctionType CreatorFunction) :
    CreatorFunction(CreatorFunction) {}

  std::unique_ptr<BackingContainerBase> createEmpty() const override {
    return CreatorFunction();
  }
  ~BackingContainerFunctionFactoryWrapper() override = default;

private:
  std::unique_ptr<BackingContainerBase> (*CreatorFunction)(void);
};

template<typename T>
class DefaultConstructibleBackingContainerFactory
  : public BackingContainerRegistryEntry {
public:
  std::unique_ptr<BackingContainerBase> createEmpty() const override {
    return std::make_unique<T>();
  }
  ~DefaultConstructibleBackingContainerFactory() override = default;
};

class BackingContainerRegistry {
public:
  template<typename Factory, typename... Arguments>
  void addContainerFactory(llvm::StringRef Name, Arguments &&... Args) {
    auto BC = std::make_unique<Factory>(std::forward<Arguments>(Args)...);
    Entries.insert_or_assign(Name, std::move(BC));
  }

  template<typename BackingCont>
  void addDefaultConstruibleFactory(llvm::StringRef Name) {
    using FactoryT = DefaultConstructibleBackingContainerFactory<BackingCont>;
    addContainerFactory<FactoryT>(Name);
  }

  BackingContainers createEmpty() const {
    BackingContainers ToReturn;
    for (const auto &Entry : Entries)
      ToReturn.add(Entry.first(), Entry.second->createEmpty());

    return ToReturn;
  }

private:
  llvm::StringMap<std::unique_ptr<BackingContainerRegistryEntry>> Entries;
};
} // namespace Model
