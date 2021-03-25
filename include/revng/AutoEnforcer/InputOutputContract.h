#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"

namespace Model {

class InputOutputContract {
public:
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

  void deduceRequirements(BackingContainersStatus &StepStatus,
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

} // namespace Model
