#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Support/Error.h"

#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/AutoEnforcer/InputOutputContract.h"
#include "revng/Support/Debug.h"
#include "revng/Support/FunctionIntrospection.h"

namespace AutoEnforcer {

namespace Detail {
using StringArrayRef = llvm::ArrayRef<std::string>;

template<typename Enforcer, typename... Args, size_t... S>
void invokeImpl(Enforcer &Enf,
                void (Enforcer::*F)(Args...),
                BackingContainers &Containers,
                const StringArrayRef &ArgsNames,
                const std::integer_sequence<size_t, S...> &) {
  (Enf.*F)(Containers.get<std::decay_t<Args>>(ArgsNames[S])...);
}

///
/// Invokes the F member function on the Enforcer Enf passing as nth argument
/// the container with the name equal to the nth element of ArgsNames
///
template<typename Enforcer, typename... Args>
void invokeEnforcingFunction(Enforcer &Enf,
                             void (Enforcer::*F)(Args...),
                             BackingContainers &Containers,
                             const llvm::ArrayRef<std::string> &ArgsNames) {
  constexpr auto
    Indexes = std::make_integer_sequence<size_t, sizeof...(Args)>();
  revng_assert(sizeof...(Args) == ArgsNames.size());
  invokeImpl(Enf, F, Containers, ArgsNames, Indexes);
}

class EnforcerWrapperBaseImpl {
public:
  virtual void run(BackingContainers &Containers) = 0;
  virtual BackingContainersStatus
  getRequirements(BackingContainersStatus &Target) const = 0;
  virtual BackingContainersStatus
  deduceResults(BackingContainersStatus &Target) const = 0;
  virtual ~EnforcerWrapperBaseImpl() = default;
  virtual std::unique_ptr<EnforcerWrapperBaseImpl> clone() const = 0;
  virtual std::vector<std::string> getRunningContainersNames() const = 0;
  virtual std::string getName() const = 0;
  virtual void dump(std::ostream &OS, size_t indents) const = 0;
};

template<typename T>
concept Printable = requires(T a) {
  { a.print(dbg, 0) };
};

template<typename Enforcer>
class EnforcerWrapperImpl : public EnforcerWrapperBaseImpl {
public:
  EnforcerWrapperImpl(Enforcer Enf,
                      std::vector<std::string> RunningContainersNames) :
    Enf(std::move(Enf)),
    RunningContainersNames(std::move(RunningContainersNames)) {}

  void run(BackingContainers &Containers) override {
    invokeEnforcingFunction(Enf,
                            &Enforcer::run,
                            Containers,
                            RunningContainersNames);
  }

  BackingContainersStatus
  getRequirements(BackingContainersStatus &Target) const override {
    const auto &Contract = Enf.getContract();
    auto Range = llvm::make_range(Contract.rbegin(), Contract.rend());
    for (const auto &Contract : Range)
      Contract.deduceRequirements(Target, RunningContainersNames);
    return Target;
  }

  BackingContainersStatus
  deduceResults(BackingContainersStatus &Target) const override {
    for (const auto &Contract : Enf.getContract())
      Contract.deduceResults(Target, RunningContainersNames);
    return Target;
  }

  ~EnforcerWrapperImpl() override = default;

  std::unique_ptr<EnforcerWrapperBaseImpl> clone() const override {
    return std::make_unique<EnforcerWrapperImpl>(*this);
  }

  std::vector<std::string> getRunningContainersNames() const override {
    return RunningContainersNames;
  }

  std::string getName() const override { return Enforcer::Name; }

  void dump(std::ostream &OS, size_t Indents) const override {
    if constexpr (Printable<Enforcer>)
      Enf.dump(OS, Indents);
  }

private:
  Enforcer Enf;
  std::vector<std::string> RunningContainersNames;
};

} // namespace Detail

class EnforcerWrapper {
public:
  template<typename Enforcer>
  static EnforcerWrapper
  makeWrapper(std::vector<std::string> RunningContainersNames) {
    return EnforcerWrapper(Enforcer(), std::move(RunningContainersNames));
  }

  EnforcerWrapper(const EnforcerWrapper &Other) : Enf(Other.Enf->clone()) {}
  EnforcerWrapper(EnforcerWrapper &&Other) = default;
  EnforcerWrapper &operator=(const EnforcerWrapper &Other);
  EnforcerWrapper &operator=(EnforcerWrapper &&Other) = default;
  ~EnforcerWrapper() = default;

  template<typename Enforcer>
  EnforcerWrapper(Enforcer Enf,
                  std::vector<std::string> RunningContainersNames) :
    Enf(makeUniqueWrapper(Enf, std::move(RunningContainersNames))) {}

  void run(BackingContainers &Container) const { Enf->run(Container); }

  BackingContainersStatus
  getRequirements(BackingContainersStatus Target) const {
    return Enf->getRequirements(Target);
  }

  BackingContainersStatus deduceResults(BackingContainersStatus Input) const {
    return Enf->deduceResults(Input);
  }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    indent(OS, Indents);
    OS << Enf->getName() << "\n";
    indent(OS, Indents + 1);
    OS << "Containers\n";
    for (const auto &Name : Enf->getRunningContainersNames()) {
      indent(OS, Indents + 2);
      OS << Name;
      OS << "\n";
    }

    Enf->dump(OS, Indents + 1);
  }

  void dump() const { dump(dbg); }

private:
  template<typename Enforcer>
  auto makeUniqueWrapper(Enforcer Enf,
                         std::vector<std::string> RunningContainersNames) {
    using Wrapper = Detail::EnforcerWrapperImpl<Enforcer>;
    return std::make_unique<Wrapper>(Enf, std::move(RunningContainersNames));
  }

  std::unique_ptr<Detail::EnforcerWrapperBaseImpl> Enf;
};

template<typename Enforcer, typename... ContainerNames>
EnforcerWrapper bindEnforcer(ContainerNames &&... Names) {
  auto NamesList = { std::forward<ContainerNames>(Names)... };
  return EnforcerWrapper::makeWrapper<Enforcer>(std::move(NamesList));
}

class Step {
public:
  BackingContainers cloneAndRun(const BackingContainersStatus &Targets);

  BackingContainersStatus
  satisfiableGoals(const BackingContainersStatus &RequiredGoals,
                   BackingContainersStatus &ToLoad) const;

  BackingContainersStatus deduceResults(BackingContainersStatus Input) const;

  void mergeBackingContainers(BackingContainers RunningContainers) {
    BackingContainer.mergeBackingContainers(std::move(RunningContainers));
  }

  const BackingContainers &getBackingContainers() const {
    return BackingContainer;
  }

  BackingContainers &getBackingContainers() { return BackingContainer; }

  void addEnforcer(EnforcerWrapper Wrapper) {
    Enforcers.push_back(std::move(Wrapper));
  }

  template<typename... EnforcerWrapperTypes>
  Step(std::string Name,
       BackingContainers BackingContainer,
       EnforcerWrapperTypes &&... EnforcerWrappers) :
    Name(std::move(Name)),
    BackingContainer(std::move(BackingContainer)),
    Enforcers({ std::forward<EnforcerWrapperTypes>(EnforcerWrappers)... }) {}

  llvm::StringRef getName() const { return Name; }

  template<typename OStream>
  void dump(OStream &OS, size_t Indents = 0) const {
    indent(OS, Indents);
    OS << "Step " << Name << ":\n";

    indent(OS, Indents + 1);
    OS << "Enforcers: \n";
    for (const auto &Enforcer : Enforcers)
      Enforcer.dump(OS, Indents + 2);

    indent(OS, Indents + 1);
    OS << "Backing containers: \n";
    BackingContainer.dump(OS, Indents + 2);
  }

  llvm::Error invalidate(const BackingContainersStatus &ToRemove);

  void dump() const debug_function { dump(dbg); }

  llvm::Error store(llvm::StringRef DirPath) const;
  llvm::Error load(llvm::StringRef DirPath);

  llvm::Expected<const BackingContainerBase *>
  safeGetContainer(llvm::StringRef ContainerName) const {
    return BackingContainer.safeGetContainer(ContainerName);
  }

  llvm::Expected<BackingContainerBase *>
  safeGetContainer(llvm::StringRef ContainerName) {
    return BackingContainer.safeGetContainer(ContainerName);
  }

private:
  static void
  removeSatisfiedGoals(BackingContainersStatus::TargetContainer &RequiredInputs,
                       const BackingContainerBase &CachedSymbols,
                       BackingContainersStatus::TargetContainer &ToLoad);

  void removeSatisfiedGoals(BackingContainersStatus &Targets,
                            BackingContainersStatus &ToLoad) const;

  std::string Name;
  BackingContainers BackingContainer;
  std::vector<EnforcerWrapper> Enforcers;
};

}; // namespace AutoEnforcer
