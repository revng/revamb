#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <string>
#include <utility>

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringMap.h"
#include "llvm/Support/Error.h"

#include "revng/ADT/Iterator.h"
#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainerRegistry.h"
#include "revng/AutoEnforcer/InvalidationEvent.hpp"
#include "revng/AutoEnforcer/Pipeline.h"
#include "revng/Support/Debug.h"

namespace AutoEnforcer {

class KindsRegisty {
public:
  using Container = llvm::SmallVector<Kind *, 3>;

  KindsRegisty(llvm::SmallVector<Kind *, 3> Kinds = {}) :
    Kinds(std::move(Kinds)) {}
  std::set<AutoEnforcerTarget>
  deduceInvalidations(const InvalidationEventBase &Event);
  void registerKind(Kind &K) { Kinds.push_back(&K); }

  auto begin() { return derefereceIterator(Kinds.begin()); }

  auto end() { return derefereceIterator(Kinds.end()); }

  auto begin() const { return derefereceIterator(Kinds.begin()); }

  auto end() const { return derefereceIterator(Kinds.end()); }

  template<typename OS>
  void dump(OS &OStream) const {
    for (const auto &K : *this)
      OStream << K.getName().str() << "\n";
  }

  void dump() const debug_function { dump(dbg); }

private:
  Container Kinds;
};

class PipelineRunner {
public:
  using iterator = std::vector<Step>::iterator;
  using const_iterator = std::vector<Step>::const_iterator;
  using InvalidationMap = llvm::StringMap<BackingContainersStatus>;

  PipelineRunner(KindsRegisty KRegistry) : KindRegistry(std::move(KRegistry)) {}

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
  void registerContainerFactory(llvm::StringRef Name, Args &&... Arguments) {
    revng_assert(not CommittedRegistry);
    using BCT = BackingContainerType;
    Registry.registerContainerFactory<BCT>(Name,
                                           std::forward<Args>(Arguments)...);
  }

  void
  registerContainerFactory(llvm::StringRef Name,
                           std::unique_ptr<BackingContainerFactory> Entry) {
    Registry.registerContainerFactory(Name, std::move(Entry));
  }

  template<typename BackingContainer>
  void registerDefaultConstructibleFactory(llvm::StringRef Name) {
    revng_assert(not CommittedRegistry);
    Registry.registerDefaultConstructibleFactory<BackingContainer>(Name);
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
  llvm::Error invalidate(const InvalidationEventBase &Event);

  llvm::Error store(llvm::StringRef DirPath) const {
    return Pipeline.store(DirPath);
  }
  llvm::Error load(llvm::StringRef DirPath) { return Pipeline.load(DirPath); }

  llvm::Expected<const BackingContainerBase *>
  safeGetContainer(llvm::StringRef StepName,
                   llvm::StringRef ContainerName) const;
  llvm::Expected<BackingContainerBase *>
  safeGetContainer(llvm::StringRef StepName, llvm::StringRef ContainerName);

  std::set<AutoEnforcerTarget>
  deduceInvalidations(const InvalidationEventBase &Event) {
    return KindRegistry.deduceInvalidations(Event);
  }

  const KindsRegisty &getKindRegistry() const { return KindRegistry; }

private:
  BackingContainerRegistry Registry;
  bool CommittedRegistry = false;
  Pipeline Pipeline;
  KindsRegisty KindRegistry;
};

class PipelineFileMapping {
private:
  std::string Step;
  std::string BackingContainer;
  std::string InputFile;

public:
  PipelineFileMapping(std::string Step,
                      std::string BackingContainer,
                      std::string InputFile) :
    Step(std::move(Step)),
    BackingContainer(std::move(BackingContainer)),
    InputFile(std::move(InputFile)) {}
  static llvm::Expected<PipelineFileMapping> parse(llvm::StringRef ToParse);
  llvm::Error load(PipelineRunner &LoadInto) const;
  llvm::Error store(const PipelineRunner &LoadInto) const;
};
} // namespace AutoEnforcer
