//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/Support/Error.h"

#include "revng/AutoEnforcer/AutoEnforcerErrors.h"
#include "revng/AutoEnforcer/BackingContainers.h"

using namespace AutoEnforcer;
using namespace llvm;
using namespace std;

BackingContainers
BackingContainers::cloneFiltered(const BackingContainersStatus &Targets) {
  BackingContainers Container;
  for (const auto &Pair : Containers) {
    const auto &ContainerName = Pair.first();
    const auto &BackingContainer = *Pair.second;

    auto ExtractedNames = Targets.contains(ContainerName) ?
                            Targets.at(ContainerName) :
                            BackingContainersStatus::TargetContainer();

    Container.add(ContainerName,
                  BackingContainer.cloneFiltered(std::move(ExtractedNames)));
  }
  revng_assert(Container.Containers.size() == Containers.size());
  return Container;
}

bool BackingContainers::contains(const AutoEnforcerTarget &Target) const {
  return llvm::any_of(Containers, [&Target](const auto &Container) {
    return Container.second->contains(Target);
  });
}

Error BackingContainers::remove(const BackingContainersStatus &ToRemove) {
  for (const auto &Target : ToRemove) {
    const auto &ContainerName = Target.first();
    const auto &NamesToRemove = Target.second;
    if (auto Ok = get(ContainerName).remove(NamesToRemove); not Ok)
      return make_error<UnknownAutoEnforcerTarget>(NamesToRemove,
                                                   ContainerName);
  }
  return Error::success();
}

void BackingContainers::intersect(BackingContainersStatus &ToIntersect) const {
  for (auto &ContainerStatus : ToIntersect) {
    const auto &ContainerName = ContainerStatus.first();
    auto &Names = ContainerStatus.second;

    const auto &BackingContainer = get(ContainerName);
    erase_if(Names, [&BackingContainer](const AutoEnforcerTarget &Target) {
      return not BackingContainer.contains(Target);
    });
  }
}

llvm::Error BackingContainers::store(StringRef Directory) const {
  for (const auto &Pair : Containers) {
    const auto &Name = Directory.str() + "/" + Pair.first().str();
    const auto &Container = Pair.second;

    if (auto Error = Container->storeToDisk(Name); !!Error)
      return Error;
  }
  return Error::success();
}

llvm::Error BackingContainers::load(StringRef Directory) {
  for (const auto &Pair : Containers) {
    const auto &Name = Directory.str() + "/" + Pair.first().str();
    const auto &Container = Pair.second;
    if (auto Error = Container->loadFromDisk(Name); !!Error)
      return Error;
  }
  return Error::success();
}

llvm::Expected<const BackingContainerBase *>
BackingContainers::safeGetContainer(llvm::StringRef ContainerName) const {
  auto It = Containers.find(ContainerName);
  if (It == Containers.end())
    return createStringError(inconvertibleErrorCode(),
                             "could not find container named %s in backing "
                             "containers",
                             ContainerName.str().c_str());

  return &*It->second;
}

llvm::Expected<BackingContainerBase *>
BackingContainers::safeGetContainer(llvm::StringRef ContainerName) {
  auto It = Containers.find(ContainerName);
  if (It == Containers.end())
    return createStringError(inconvertibleErrorCode(),
                             "could not find container named %s in backing "
                             "containers",
                             ContainerName.str().c_str());

  return &*It->second;
}
