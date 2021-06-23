#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <string>

#include "llvm/ADT/ArrayRef.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"

namespace AutoEnforcer {

class InputOutputContract {
public:
  using TargetContainer = BackingContainersStatus::TargetContainer;
  constexpr InputOutputContract(const Kind &Source,
                                KindExactness InputContract,
                                size_t EnforcerArgumentSourceIndex,
                                const Kind &Target,
                                size_t EnforcerArgumentTargetIndex = 0,
                                bool PreservedInput = false) :
    Source(&Source),
    InputContract(InputContract),
    Target(&Target),
    EnforcerArgumentSourceIndex(EnforcerArgumentSourceIndex),
    EnforcerArgumentTargetIndex(EnforcerArgumentTargetIndex),
    PreservedInput(PreservedInput) {}

  constexpr InputOutputContract(const Kind &Source,
                                KindExactness InputContract,
                                size_t EnforcerArgumentSourceIndex = 0,
                                bool PreservedInput = false) :
    Source(&Source),
    InputContract(InputContract),
    Target(nullptr),
    EnforcerArgumentSourceIndex(EnforcerArgumentSourceIndex),
    EnforcerArgumentTargetIndex(EnforcerArgumentSourceIndex),
    PreservedInput(PreservedInput) {}

  void deduceResults(BackingContainersStatus &StepStatus,
                     llvm::ArrayRef<std::string> ContainerNames) const;

  void deduceResults(BackingContainersStatus &StepStatus,
                     TargetContainer &Results,
                     llvm::ArrayRef<std::string> ContainerNames) const;

  void deduceResults(BackingContainersStatus &StepStatus,
                     BackingContainersStatus &Results,
                     llvm::ArrayRef<std::string> ContainerNames) const;

  void deduceRequirements(BackingContainersStatus &StepStatus,
                          llvm::ArrayRef<std::string> ContainerNames) const;

  void deduceRequirements(BackingContainersStatus &StepStatus,
                          TargetContainer &Results,
                          llvm::ArrayRef<std::string> ContainerNames) const;

  void deduceRequirements(BackingContainersStatus &StepStatus,
                          BackingContainersStatus &Results,
                          llvm::ArrayRef<std::string> ContainerNames) const;

  bool forwardMatches(const BackingContainersStatus &Status,
                      llvm::ArrayRef<std::string> ContainerNames) const;
  bool backwardMatches(const BackingContainersStatus &Status,
                       llvm::ArrayRef<std::string> ContainerNames) const;

  void insertDefaultInput(BackingContainersStatus &Status,
                          llvm::ArrayRef<std::string> ContainerNames) const;

private:
  void forward(AutoEnforcerTarget &Input) const;
  bool forwardMatches(const AutoEnforcerTarget &Input) const;
  void forwardGranularity(AutoEnforcerTarget &Input) const;

  ///
  /// Target fixed -> Output must be exactly Target
  /// Target same as Source, Source derived from base ->  Most strict between
  /// source and target Target same as source, source exactly base -> base
  ///
  void backward(AutoEnforcerTarget &Output) const;
  KindExactness backwardInputContract(const AutoEnforcerTarget &Output) const;
  void backwardGranularity(AutoEnforcerTarget &Output) const;
  const Kind &backwardInputKind(const AutoEnforcerTarget &Output) const;
  bool backwardMatches(const AutoEnforcerTarget &Output) const;

  const Kind *Source;
  KindExactness InputContract;
  const Kind *Target;
  size_t EnforcerArgumentSourceIndex;
  size_t EnforcerArgumentTargetIndex;
  bool PreservedInput;
};

class AtomicContract {
public:
  using TargetContainer = BackingContainersStatus::TargetContainer;
  AtomicContract(std::initializer_list<InputOutputContract> Contract) :
    Contract(std::move(Contract)) {}

  AtomicContract(llvm::SmallVector<InputOutputContract, 2> Contract) :
    Contract(std::move(Contract)) {}

  AtomicContract(llvm::ArrayRef<AtomicContract> Contracts) {
    for (const auto &C : Contracts)
      for (const auto &Entry : C.Contract)
        Contract.push_back(Entry);
  }

  AtomicContract(const Kind &Source,
                 KindExactness InputContract,
                 size_t EnforcerArgumentSourceIndex,
                 const Kind &Target,
                 size_t EnforcerArgumentTargetIndex = 0,
                 bool PreservedInput = false) :
    Contract({ InputOutputContract(Source,
                                   InputContract,
                                   EnforcerArgumentSourceIndex,
                                   Target,
                                   EnforcerArgumentTargetIndex,
                                   PreservedInput) }) {}

  AtomicContract(const Kind &Source,
                 KindExactness InputContract,
                 size_t EnforcerArgumentSourceIndex = 0,
                 bool PreservedInput = false) :
    Contract({ InputOutputContract(Source,
                                   InputContract,
                                   EnforcerArgumentSourceIndex,
                                   PreservedInput) }) {}

  void deduceRequirements(BackingContainersStatus &StepStatus,
                          llvm::ArrayRef<std::string> ContainerNames) const;
  void deduceResults(BackingContainersStatus &StepStatus,
                     llvm::ArrayRef<std::string> ContainerNames) const;

  bool forwardMatches(const BackingContainersStatus &Status,
                      llvm::ArrayRef<std::string> ContainerNames) const;
  bool backwardMatches(const BackingContainersStatus &Status,
                       llvm::ArrayRef<std::string> ContainerNames) const;

private:
  llvm::SmallVector<InputOutputContract, 2> Contract;
};

} // namespace AutoEnforcer
