//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/Pipeline.h"

using namespace std;
using namespace llvm;
using namespace AutoEnforcer;

namespace AutoEnforcer {
Error Pipeline::getObjectives(const BackingContainersStatus &Targets,
                              BackingContainersStatus &ToLoad,
                              std::vector<PipelineExecutionEntry> &ToExec) {
  BackingContainersStatus PartialGoals = Targets;
  for (Step &CurrentStep : make_range(Steps.rbegin(), Steps.rend())) {
    if (PartialGoals.empty())
      break;

    PartialGoals = CurrentStep.satisfiableGoals(Targets, ToLoad);
    ToExec.emplace_back(CurrentStep, PartialGoals);
  }

  if (not PartialGoals.empty())
    return make_error<UnsatisfiableRequestError>(Targets, PartialGoals);
  reverse(ToExec.begin(), ToExec.end());
  return Error::success();
}

using StatusMap = llvm::StringMap<BackingContainersStatus>;

Error Pipeline::getInvalidations(StatusMap &Invalidated) const {
  for (size_t I = 0; I < Steps.size() - 1; I++) {
    const Step &S = Steps[I];
    const Step &NextS = Steps[I + 1];
    BackingContainersStatus &Inputs = Invalidated[S.getName()];

    BackingContainersStatus &Outputs = Invalidated[NextS.getName()];

    auto Deduced = S.deduceResults(Inputs);
    NextS.getBackingContainers().intersect(Deduced);
    Outputs.merge(Deduced);
  }
  return Error::success();
}

Error Pipeline::run(const BackingContainersStatus &Targets) {
  optional<BackingContainers> CurrentContainer = std::nullopt;

  BackingContainersStatus ToLoad;
  vector<PipelineExecutionEntry> ToExec;
  if (auto Error = getObjectives(Targets, ToLoad, ToExec); Error)
    return Error;

  for (auto &StepGoalsPairs : ToExec) {
    auto &[Step, Goals] = StepGoalsPairs;
    if (CurrentContainer.has_value()) {
      Step->mergeBackingContainers(std::move(*CurrentContainer));
      // todo save on disk
    }

    CurrentContainer = Step->cloneAndRun(ToLoad);
  }
  return Error::success();
}

const Step &Pipeline::operator[](llvm::StringRef Name) const {
  return *find(Name);
}

Step &Pipeline::operator[](llvm::StringRef Name) {
  return *find(Name);
}

Pipeline::iterator Pipeline::find(llvm::StringRef Name) {
  return find_if(*this,
                 [&Name](const auto &Step) { return Step.getName() == Name; });
}

Pipeline::const_iterator Pipeline::find(llvm::StringRef Name) const {
  return find_if(*this,
                 [&Name](const auto &Step) { return Step.getName() == Name; });
}

Error Pipeline::invalidate(const StatusMap &Invalidations) {
  for (const auto &Step : Invalidations) {
    const auto &StepName = Step.first();
    const auto &ToRemove = Step.second;
    if (auto Error = find(StepName)->invalidate(ToRemove); Error)
      return Error;
  }
  return Error::success();
}

} // namespace AutoEnforcer
