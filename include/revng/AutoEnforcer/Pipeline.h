#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <initializer_list>
#include <optional>
#include <vector>

#include "llvm/ADT/STLExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/AutoEnforcer/AutoEnforcerErrors.h"
#include "revng/AutoEnforcer/Step.h"

namespace AutoEnforcer {

class PipelineExecutionEntry {
public:
  PipelineExecutionEntry(Step &ToExecute, BackingContainersStatus Objectives) :
    ToExecute(&ToExecute), Objectives(std::move(Objectives)) {}

  Step *ToExecute;
  BackingContainersStatus Objectives;
};

class Pipeline {
public:
  using iterator = std::vector<Step>::iterator;
  using const_iterator = std::vector<Step>::const_iterator;

  void add(Step &&NewStep) { Steps.emplace_back(std::move(NewStep)); }

  Step &operator[](size_t Index) { return Steps[Index]; }
  const Step &operator[](size_t Index) const { return Steps[Index]; }

  const Step &back() const { return Steps.back(); }

  Step &back() { return Steps.back(); }

  const BackingContainers &getStartingContainers() const {
    return Steps.front().getBackingContainers();
  }
  BackingContainers &getStartingContainers() {
    return Steps.front().getBackingContainers();
  }

  template<typename BackingContainerType>
  const BackingContainerType &getStartingContainer(llvm::StringRef Name) const {
    return getStartingContainers().template get<BackingContainerType>(Name);
  }

  template<typename BackingContainerType>
  BackingContainerType &getStartingContainer(llvm::StringRef Name) {
    return getStartingContainers().template get<BackingContainerType>(Name);
  }

  const BackingContainers &getFinalContainers() const {
    return Steps.back().getBackingContainers();
  }
  BackingContainers &getFinalContainers() {
    return Steps.back().getBackingContainers();
  }

  template<typename BackingContainerType>
  const BackingContainerType &getFinalContainer(llvm::StringRef Name) const {
    return getFinalContainers().template get<BackingContainerType>(Name);
  }

  template<typename BackingContainerType>
  BackingContainerType &getFinalContainer(llvm::StringRef Name) {
    return getFinalContainers().template get<BackingContainerType>(Name);
  }

  llvm::Error
  getInvalidations(llvm::StringMap<BackingContainersStatus> &Invalidated) const;

  ///
  /// \return the step with the provided name
  ///
  const Step &operator[](llvm::StringRef Name) const;
  ///
  /// \return the step with the provided name
  ///
  Step &operator[](llvm::StringRef Name);

  ///
  /// \return the step with the provided name
  ///
  const_iterator find(llvm::StringRef Name) const;
  ///
  /// \return the step with the provided name
  ///
  iterator find(llvm::StringRef Name);

  iterator begin() { return Steps.begin(); }
  iterator end() { return Steps.end(); }

  const_iterator begin() const { return Steps.begin(); }
  const_iterator end() const { return Steps.end(); }

  llvm::Error getObjectives(const BackingContainersStatus &Targets,
                            BackingContainersStatus &ToLoad,
                            std::vector<PipelineExecutionEntry> &ToExec);
  llvm::Error run(const BackingContainersStatus &Targets);

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    for (const auto &Step : Steps)
      Step.dump(OS, Indents);
  }

  void dump() const debug_function { dump(dbg); }

  llvm::Error
  invalidate(const llvm::StringMap<BackingContainersStatus> &Invalidations);

private:
  std::vector<Step> Steps;
};
} // namespace AutoEnforcer
