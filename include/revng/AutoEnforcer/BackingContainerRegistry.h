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

namespace AutoEnforcer {
class BackingContainerFactory {
public:
  virtual std::unique_ptr<BackingContainerBase> createEmpty() const = 0;
  virtual ~BackingContainerFactory() = default;
};

class BackingContainerFunctionFactoryWrapper : public BackingContainerFactory {
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
  : public BackingContainerFactory {
public:
  std::unique_ptr<BackingContainerBase> createEmpty() const override {
    return std::make_unique<T>();
  }
  ~DefaultConstructibleBackingContainerFactory() override = default;
};

class BackingContainerRegistry {
public:
  template<typename Factory, typename... Arguments>
  void registerContainerFactory(llvm::StringRef Name, Arguments &&... Args) {
    auto BC = std::make_unique<Factory>(std::forward<Arguments>(Args)...);
    Entries.insert_or_assign(Name, std::move(BC));
  }

  void
  registerContainerFactory(llvm::StringRef Name,
                           std::unique_ptr<BackingContainerFactory> Entry) {
    Entries.insert_or_assign(Name, std::move(Entry));
  }

  template<typename BackingCont>
  void registerDefaultConstructibleFactory(llvm::StringRef Name) {
    using FactoryT = DefaultConstructibleBackingContainerFactory<BackingCont>;
    registerContainerFactory<FactoryT>(Name);
  }

  BackingContainers createEmpty() const {
    BackingContainers ToReturn;
    for (const auto &Entry : Entries)
      ToReturn.add(Entry.first(), Entry.second->createEmpty());

    return ToReturn;
  }

private:
  llvm::StringMap<std::unique_ptr<BackingContainerFactory>> Entries;
};
} // namespace AutoEnforcer
