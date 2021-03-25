//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/Step.h"
#include "revng/Support/Debug.h"

using namespace llvm;
using namespace std;
using namespace Model;

EnforcerWrapper &EnforcerWrapper::operator=(const EnforcerWrapper &Other) {
  if (this == &Other)
    return *this;

  Enf = Other.Enf->clone();
  return *this;
}

BackingContainersStatus
Step::satisfiableGoals(const BackingContainersStatus &RequiredGoals,
                       BackingContainersStatus &ToLoad) const {

  BackingContainersStatus Targets = RequiredGoals;
  for (const auto &Enforcer :
       llvm::make_range(Enforcers.rbegin(), Enforcers.rend())) {
    Targets = Enforcer.getRequirements(Targets);
    removeSatisfiedGoals(Targets, ToLoad);
  }

  return Targets;
}

BackingContainers Step::cloneAndRun(const BackingContainersStatus &Targets) {
  auto RunningContainers = BackingContainer.cloneFiltered(Targets);
  for (auto &Enforcer : Enforcers)
    Enforcer.run(RunningContainers);
  return RunningContainers;
}

using TargetContainer = BackingContainersStatus::TargetContainer;

void Step::removeSatisfiedGoals(TargetContainer &RequiredInputs,
                                const BackingContainerBase &CachedSymbols,
                                TargetContainer &ToLoad) {

  const auto IsCached =
    [&ToLoad, &CachedSymbols](const AutoEnforcerTarget &Target) -> bool {
    bool MustBeLoaded = CachedSymbols.contains(Target);
    if (MustBeLoaded)
      ToLoad.emplace_back(Target);
    return MustBeLoaded;
  };

  llvm::erase_if(RequiredInputs, IsCached);
}

void Step::removeSatisfiedGoals(BackingContainersStatus &Targets,
                                BackingContainersStatus &ToLoad) const {
  for (auto &RequiredInputsFromContainer : Targets) {
    llvm::StringRef ContainerName = RequiredInputsFromContainer.first();
    auto &RequiredInputs = RequiredInputsFromContainer.second;
    auto &ToLoadFromCurrentContainer = ToLoad[ContainerName];
    removeSatisfiedGoals(RequiredInputs,
                         BackingContainer.get(ContainerName),
                         ToLoadFromCurrentContainer);
  }
}

BackingContainersStatus
Step::deduceResults(BackingContainersStatus Input) const {
  for (const auto &Enforcer : Enforcers)
    Input = Enforcer.deduceResults(Input);
  return Input;
}

void BackingContainersStatus::merge(const BackingContainersStatus &Other) {
  for (const auto &Container : Other.ContainersStatus) {
    const auto &BackingContainerName = Container.first();
    const auto &BackingContainerSymbols = Container.second;
    auto &ToMergeIn = ContainersStatus[BackingContainerName];

    copy(BackingContainerSymbols, back_inserter(ToMergeIn));
  }
}

Error Step::invalidate(const BackingContainersStatus &ToRemove) {
  return getBackingContainers().remove(ToRemove);
}
