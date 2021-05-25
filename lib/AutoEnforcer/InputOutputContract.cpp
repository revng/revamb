//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/AutoEnforcer/InputOutputContract.h"

using namespace AutoEnforcer;
using namespace llvm;
using namespace std;

void InputOutputContract::deduceResults(BackingContainersStatus &StepStatus,
                                        ArrayRef<string> ContainerNames) const {
  auto &SourceContainerTargets = StepStatus
    [ContainerNames[EnforcerArgumentSourceIndex]];
  auto &OutputContainerTarget = StepStatus
    [ContainerNames[EnforcerArgumentTargetIndex]];

  // We need a temporary storage for the targets because the source and output
  // may be the same container and thus we would erase the just inserted targets
  // as well.
  BackingContainersStatus::TargetContainer Tmp;

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return forwardMatches(Input);
  };

  copy_if(SourceContainerTargets, back_inserter(Tmp), Matches);
  if (not PreservedInput)
    erase_if(SourceContainerTargets, Matches);

  for (AutoEnforcerTarget &Target : Tmp)
    forward(Target);

  copy(Tmp, back_inserter(OutputContainerTarget));
}

void InputOutputContract::deduceRequirements(BackingContainersStatus &Status,
                                             ArrayRef<string> Names) const {

  auto &SourceContainerTargets = Status[Names[EnforcerArgumentSourceIndex]];
  auto &OutputContainerTarget = Status[Names[EnforcerArgumentTargetIndex]];

  // We need a temporary storage for the targets because we source and output
  // may be the same container and thus we would erase the just inserted targets
  // as well.
  BackingContainersStatus::TargetContainer Tmp;

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return backwardMatches(Input) or (PreservedInput and forwardMatches(Input));
  };

  copy_if(OutputContainerTarget, back_inserter(Tmp), Matches);
  erase_if(OutputContainerTarget, Matches);

  for (AutoEnforcerTarget &Out : Tmp)
    backward(Out);

  copy(Tmp, back_inserter(SourceContainerTargets));
}

void InputOutputContract::forward(AutoEnforcerTarget &Input) const {
  // A enforcer cannot yield a instance with multiple kinds when going
  // forward.
  revng_assert(Input.kindExactness() == KindExactness::Exact);

  const auto *OutputKind = Target != nullptr ? Target : &Input.getKind();
  Input.setKind(*OutputKind);
  forwardGranularity(Input);
}

bool InputOutputContract::forwardMatches(const AutoEnforcerTarget &In) const {
  switch (InputContract) {
  case KindExactness::DerivedFrom:
    return Source->ancestorOf(In.getKind());
  case KindExactness::Exact:
    return &In.getKind() == Source;
  }
  return false;
}

void InputOutputContract::backward(AutoEnforcerTarget &Output) const {
  if (not backwardMatches(Output))
    return;

  Output.setKind(backwardInputKind(Output));
  Output.setKindExactness(backwardInputContract(Output));
  backwardGranularity(Output);
}

KindExactness
InputOutputContract::backwardInputContract(const AutoEnforcerTarget &O) const {
  if (Target != nullptr)
    return InputContract;

  if (InputContract == KindExactness::Exact)
    return KindExactness::Exact;

  return O.kindExactness();
}

void InputOutputContract::forwardGranularity(AutoEnforcerTarget &Input) const {
  const auto *InputGranularity = Source->get();
  const auto *OutputGranularity = Target != nullptr ? Target->get() :
                                                      InputGranularity;
  if (InputGranularity == OutputGranularity)
    return;

  // if the output is at a greater level of depth of the hierarchy
  // than the input, for each level of difference add a granularity to the
  // target.
  //
  if (InputGranularity->ancestorOf(*OutputGranularity)) {
    while (InputGranularity != OutputGranularity) {
      Input.addGranularity();
      OutputGranularity = OutputGranularity->getParent();
    }
    return;
  }

  // If the output is less fined grained than the input drop levels of
  // granularity until they have the same.
  if (OutputGranularity->ancestorOf(*InputGranularity)) {
    while (OutputGranularity != InputGranularity) {
      // if you are decreasing the granularity, you must have at your disposal
      // all symbols.
      revng_assert(Input.getQuantifiers().back().isAll());
      Input.dropGranularity();
      InputGranularity = InputGranularity->getParent();
    }
    return;
  }

  revng_abort("Unreachable");
}

void InputOutputContract::backwardGranularity(AutoEnforcerTarget &Out) const {
  const auto *InputGranularity = Source->get();
  const auto *OutputGranularity = Target != nullptr ? Target->get() :
                                                      InputGranularity;
  if (InputGranularity == OutputGranularity)
    return;

  if (OutputGranularity->ancestorOf(*InputGranularity)) {
    while (InputGranularity != OutputGranularity) {
      Out.addGranularity();
      InputGranularity = InputGranularity->getParent();
    }
    return;
  }

  if (InputGranularity->ancestorOf(*OutputGranularity)) {
    while (InputGranularity != OutputGranularity) {
      // if you are decreasing the granularity, you must have at your disposal
      // all symbols.
      Out.dropGranularity();
      OutputGranularity = OutputGranularity->getParent();
    }
    return;
  }

  revng_abort("Unreachable");
}

const Kind &
InputOutputContract::backwardInputKind(const AutoEnforcerTarget &Output) const {
  // If the enforcer requires exactly a particular kind, return that one
  if (InputContract == KindExactness::Exact)
    return *Source;

  if (Target != nullptr)
    return *Source;

  // Otherwise return the most restricting between input requirement and
  // output. We have already know that one derives the other.
  if (Source->ancestorOf(Output.getKind()))
    return Output.getKind();

  return *Source;
}

bool InputOutputContract::backwardMatches(const AutoEnforcerTarget &Out) const {
  if (Target != nullptr)
    return &Out.getKind() == Target;

  switch (InputContract) {
  case KindExactness::DerivedFrom:
    return Source->ancestorOf(Out.getKind())
           or (Out.kindExactness() == KindExactness::DerivedFrom
               and Out.getKind().ancestorOf(*Source));
  case KindExactness::Exact:
    return Out.getKind().ancestorOf(*Source);
  }
}
