#include "revng/AutoEnforcer/AutoEnforcer.h"

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

using namespace std;
using namespace llvm;
using namespace AutoEnforcer;

using InvalidationMap = StringMap<BackingContainersStatus>;
Expected<InvalidationMap>
PipelineRunner::getInvalidations(const AutoEnforcerTarget &Target) const {
  llvm::StringMap<BackingContainersStatus> Invalidations;
  for (const auto &Step : Pipeline)
    for (const auto &Container : Step.getBackingContainers()) {

      if (Container.second->contains(Target)) {
        Invalidations[Step.getName()].add(Container.first(), Target);
      }
    }
  if (auto Error = deduceInvalidations(Invalidations); Error)
    return move(Error);

  return move(Invalidations);
}

Error PipelineRunner::invalidate(const AutoEnforcerTarget &Target) {
  auto Invalidations = getInvalidations(Target);
  if (not Invalidations)
    return Invalidations.takeError();
  return Pipeline.invalidate(*Invalidations);
}

Expected<const BackingContainerBase *>
PipelineRunner::safeGetContainer(StringRef StepName,
                                 StringRef ContainerName) const {
  auto It = find_if(*this, [&StepName](const Step &S) {
    return StepName == S.getName();
  });
  if (It == end())
    return createStringError(inconvertibleErrorCode(),
                             "Could not find step with name %s",
                             StepName.str().c_str());
  return It->safeGetContainer(ContainerName);
}

Expected<BackingContainerBase *>
PipelineRunner::safeGetContainer(StringRef StepName, StringRef ContainerName) {
  auto It = find_if(*this, [&StepName](const Step &S) {
    return StepName == S.getName();
  });
  if (It == end())
    return createStringError(inconvertibleErrorCode(),
                             "Could not find step with name %s",
                             StepName.str().c_str());
  return It->safeGetContainer(ContainerName);
}

llvm::Expected<PipelineFileMapping>
PipelineFileMapping::parse(StringRef ToParse) {
  SmallVector<StringRef, 3> Splitted;
  ToParse.split(Splitted, ':', 2);

  if (Splitted.size() != 3)
    return createStringError(inconvertibleErrorCode(),
                             "could not parse %s into three strings "
                             "step:container:inputfile",
                             ToParse.str().c_str());

  return PipelineFileMapping(Splitted[0], Splitted[1], Splitted[2]);
}

Error PipelineFileMapping::load(PipelineRunner &LoadInto) const {
  auto MaybeContainer = LoadInto.safeGetContainer(Step, BackingContainer);
  if (!MaybeContainer)
    return MaybeContainer.takeError();

  return (**MaybeContainer).loadFromDisk(InputFile);
}

Error PipelineFileMapping::store(const PipelineRunner &LoadInto) const {
  auto MaybeContainer = LoadInto.safeGetContainer(Step, BackingContainer);
  if (!MaybeContainer)
    return MaybeContainer.takeError();

  return (**MaybeContainer).storeToDisk(InputFile);
}

set<AutoEnforcerTarget>
KindsRegisty::deduceInvalidations(const InvalidationEventBase &Event) {
  set<AutoEnforcerTarget> ToReturn;
  for (const auto &Kind : Kinds) {

    set<GranularityList> Paths;
    Kind->deduceInvalidations(Event, Paths);
    for (auto &Path : Paths) {
      AutoEnforcerTarget Target(Path, *Kind, KindExactness::DerivedFrom);
      ToReturn.insert(std::move(Target));
    }
  }

  return ToReturn;
}

llvm::Error PipelineRunner::invalidate(const InvalidationEventBase &Event) {
  auto Invalidations = deduceInvalidations(Event);

  for (const auto &Invalidation : Invalidations) {
    auto Error = invalidate(Invalidation);
    if (Error)
      return Error;
  }

  return Error::success();
}
