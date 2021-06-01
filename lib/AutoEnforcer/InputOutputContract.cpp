//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/STLExtras.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/InputOutputContract.h"

using namespace AutoEnforcer;
using namespace llvm;
using namespace std;

void InputOutputContract::deduceResults(BackingContainersStatus &StepStatus,
                                        ArrayRef<string> Names) const {
  auto &OutputContainerTarget = StepStatus[Names[EnforcerArgumentTargetIndex]];

  TargetContainer Tmp;
  deduceResults(StepStatus, Tmp, Names);

  copy(Tmp, back_inserter(OutputContainerTarget));
}

void InputOutputContract::deduceResults(BackingContainersStatus &StepStatus,
                                        BackingContainersStatus &Results,
                                        ArrayRef<string> Names) const {
  auto &OutputContainerTarget = Results[Names[EnforcerArgumentTargetIndex]];

  deduceResults(StepStatus, OutputContainerTarget, Names);
}

void InputOutputContract::deduceResults(BackingContainersStatus &StepStatus,
                                        TargetContainer &Results,
                                        ArrayRef<string> Names) const {
  auto &SourceContainerTargets = StepStatus[Names[EnforcerArgumentSourceIndex]];

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return forwardMatches(Input);
  };

  copy_if(SourceContainerTargets, back_inserter(Results), Matches);
  if (not PreservedInput)
    erase_if(SourceContainerTargets, Matches);

  for (AutoEnforcerTarget &Target : Results)
    forward(Target);
}

void InputOutputContract::deduceRequirements(BackingContainersStatus &Status,
                                             ArrayRef<string> Names) const {

  auto &SourceContainerTargets = Status[Names[EnforcerArgumentSourceIndex]];

  TargetContainer Tmp;
  deduceRequirements(Status, Tmp, Names);

  copy(Tmp, back_inserter(SourceContainerTargets));
}

void InputOutputContract::deduceRequirements(BackingContainersStatus &Status,
                                             BackingContainersStatus &Results,
                                             ArrayRef<string> Names) const {

  auto &SourceContainerTargets = Results[Names[EnforcerArgumentSourceIndex]];
  deduceRequirements(Status, SourceContainerTargets, Names);
}

void InputOutputContract::deduceRequirements(BackingContainersStatus &Status,
                                             TargetContainer &Results,
                                             ArrayRef<string> Names) const {

  auto &OutputContainerTarget = Status[Names[EnforcerArgumentTargetIndex]];

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return backwardMatches(Input) or (PreservedInput and forwardMatches(Input));
  };

  copy_if(OutputContainerTarget, back_inserter(Results), Matches);
  erase_if(OutputContainerTarget, Matches);

  for (AutoEnforcerTarget &Out : Results)
    backward(Out);
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
  const auto *InputGranularity = Source->Granularity;
  const auto *OutputGranularity = Target != nullptr ? Target->Granularity :
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
  const auto *InputGranularity = Source->Granularity;
  const auto *OutputGranularity = Target != nullptr ? Target->Granularity :
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

using BCS = BackingContainersStatus;
bool InputOutputContract::forwardMatches(const BCS &StepStatus,
                                         ArrayRef<string> Names) const {
  auto It = StepStatus.find(Names[EnforcerArgumentSourceIndex]);
  if (It == StepStatus.end())
    return false;
  const auto &SourceContainerTargets = It->second;

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return forwardMatches(Input);
  };

  return any_of(SourceContainerTargets, Matches);
}

bool InputOutputContract::backwardMatches(const BCS &StepStatus,
                                          ArrayRef<string> Names) const {
  auto It = StepStatus.find(Names[EnforcerArgumentTargetIndex]);
  if (It == StepStatus.end())
    return false;
  const auto &OutputContainerTarget = It->second;

  const auto Matches = [this](const AutoEnforcerTarget &Input) {
    return backwardMatches(Input) or (PreservedInput and forwardMatches(Input));
  };

  return any_of(OutputContainerTarget, Matches);
}

void InputOutputContract::insertDefaultInput(BCS &Status,
                                             ArrayRef<string> Names) const {
  auto &SourceContainerTargets = Status[Names[EnforcerArgumentSourceIndex]];

  llvm::SmallVector<AutoEnforcerQuantifier, 3>
    Quantifiers(Source->Granularity->depth() + 1, AutoEnforcerQuantifier());

  AutoEnforcerTarget Target(move(Quantifiers), *Source, InputContract);
  SourceContainerTargets.push_back(move(Target));
}

bool AtomicContract::forwardMatches(const BCS &Status,
                                    llvm::ArrayRef<std::string> Names) const {
  return all_of(Contract, [&Status, &Names](const auto &C) {
    return C.forwardMatches(Status, Names);
  });
}

bool AtomicContract::backwardMatches(const BCS &Status,
                                     llvm::ArrayRef<std::string> Names) const {
  return any_of(Contract, [&Status, &Names](const auto &C) {
    return C.backwardMatches(Status, Names);
  });
}

void AtomicContract::deduceRequirements(BackingContainersStatus &StepStatus,
                                        ArrayRef<string> Names) const {
  if (not backwardMatches(StepStatus, Names)) {
    return;
  }

  BackingContainersStatus Results;
  for (const auto &C : llvm::reverse(Contract)) {
    if (C.backwardMatches(StepStatus, Names))
      C.deduceRequirements(StepStatus, Results, Names);
    else
      C.insertDefaultInput(Results, Names);
  }

  StepStatus.merge(Results);
  StepStatus.removeDupplicates();
}

void AtomicContract::deduceResults(BackingContainersStatus &StepStatus,
                                   ArrayRef<string> Names) const {
  if (not forwardMatches(StepStatus, Names))
    return;

  BackingContainersStatus Results;
  for (const auto &C : Contract)
    C.deduceResults(StepStatus, Results, Names);

  StepStatus.merge(Results);
  StepStatus.removeDupplicates();
}
