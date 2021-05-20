#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/StringMap.h"
#include "llvm/Support/Error.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainerRegistry.h"
#include "revng/AutoEnforcer/Pipeline.h"
#include "revng/Support/Debug.h"

namespace AutoEnforcer {
class PipelineRunner {
public:
  using iterator = std::vector<Step>::iterator;
  using const_iterator = std::vector<Step>::const_iterator;
  using InvalidationMap = llvm::StringMap<BackingContainersStatus>;

  template<typename... EnforcerWrappers>
  void addStep(std::string StepName, EnforcerWrappers &&... EnfWrappers) {
    CommittedRegistry = true;
    Pipeline.add(Step(std::move(StepName),
                      Registry.createEmpty(),
                      std::forward<EnforcerWrappers>(EnfWrappers)...));
  }

  llvm::Error deduceInvalidations(InvalidationMap &Invalidations) const {
    return Pipeline.getInvalidations(Invalidations);
  }

  const BackingContainers &getStartingContainers() const {
    return Pipeline.getStartingContainers();
  }

  BackingContainers &getStartingContainers() {
    return Pipeline.getStartingContainers();
  }

  template<typename BackingContainerType>
  const BackingContainerType &getStartingContainer(llvm::StringRef Name) const {
    return Pipeline.getStartingContainer<BackingContainerType>(Name);
  }

  template<typename BackingContainerType>
  BackingContainerType &getStartingContainer(llvm::StringRef Name) {
    return Pipeline.getStartingContainer<BackingContainerType>(Name);
  }

  const BackingContainers &getFinalContainers() const {
    return Pipeline.getFinalContainers();
  }
  BackingContainers &getFinalContainers() {
    return Pipeline.getFinalContainers();
  }

  template<typename BackingContainerType>
  const BackingContainerType &getFinalContainer(llvm::StringRef Name) const {
    return Pipeline.getFinalContainer<BackingContainerType>(Name);
  }

  template<typename BackingContainerType>
  BackingContainerType &getFinalContainer(llvm::StringRef Name) {
    return Pipeline.getFinalContainer<BackingContainerType>(Name);
  }

  llvm::Error run(const BackingContainersStatus &Targets) {
    return Pipeline.run(Targets);
  }

  template<typename BackingContainerType, typename... Args>
  void addContainerFactory(llvm::StringRef Name, Args &&... Arguments) {
    revng_assert(not CommittedRegistry);
    Registry.addContainerFactory<BackingContainerType,
                                 Args...>(Name,
                                          std::forward<Args>(Arguments)...);
  }

  void
  addContainerFactory(llvm::StringRef Name,
                      std::unique_ptr<BackingContainerRegistryEntry> Entry) {
    Registry.addContainerFactory(Name, std::move(Entry));
  }

  template<typename BackingContainer>
  void addDefaultConstruibleFactory(llvm::StringRef Name) {
    revng_assert(not CommittedRegistry);
    Registry.addDefaultConstruibleFactory<BackingContainer>(Name);
  }

  iterator begin() { return Pipeline.begin(); }
  iterator end() { return Pipeline.end(); }

  const_iterator begin() const { return Pipeline.begin(); }
  const_iterator end() const { return Pipeline.end(); }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    Pipeline.dump(OS, Indents);
  }
  void dump() const debug_function { dump(dbg); }
  Step &operator[](size_t Index) { return Pipeline[Index]; }
  const Step &operator[](size_t Index) const { return Pipeline[Index]; }

  const Step &back() const { return Pipeline.back(); }
  Step &back() { return Pipeline.back(); }

  llvm::Expected<InvalidationMap>
  getInvalidations(const AutoEnforcerTarget &Target) const;

  llvm::Error invalidate(const AutoEnforcerTarget &Target);

private:
  BackingContainerRegistry Registry;
  bool CommittedRegistry = false;
  Pipeline Pipeline;
};
} // namespace AutoEnforcer
