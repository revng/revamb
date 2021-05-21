#include <optional>

#include "llvm/ADT/StringRef.h"

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/AutoEnforcer/PipelineLoader.h"

using namespace AutoEnforcer;
using namespace std;
using namespace llvm;

llvm::Error
PipelineLoader::parseStepDeclaration(PipelineRunner &Runner,
                                     const StepDeclaration &Declaration) const {
  Runner.addStep(Declaration.Name);
  for (const auto &Invocation : Declaration.Enforcers)
    if (auto error = parseInvocation(Runner.back(), Invocation); !!error)
      return error;

  return Error::success();
}

llvm::Error
PipelineLoader::parseLLVMPass(Step &Step,
                              const EnforcerInvocation &Invocation) const {

  LLVMEnforcer ToInsert;
  for (const auto &PassName : Invocation.Passess) {
    auto It = KnownLLVMEnforcerTypes.find(PassName);
    if (It == KnownLLVMEnforcerTypes.end())
      return createStringError(inconvertibleErrorCode(),
                               "No known LLVM pass with provided name");

    auto &Entry = It->second;
    ToInsert.addPass(Entry());
  }
  Step.addEnforcer(
    EnforcerWrapper(std::move(ToInsert), Invocation.UsedContainers));

  return Error::success();
}

llvm::Error
PipelineLoader::parseInvocation(Step &Step,
                                const EnforcerInvocation &Invocation) const {
  if (Invocation.Name == "LLVMEnforcer")
    return parseLLVMPass(Step, Invocation);

  auto It = KnownEnforcersTypes.find(Invocation.Name);
  if (It == KnownEnforcersTypes.end())
    return createStringError(inconvertibleErrorCode(),
                             "No known enforcer with provided name");
  auto &Entry = It->second;
  Step.addEnforcer(Entry(Invocation.UsedContainers));
  return Error::success();
}

llvm::Error PipelineLoader::parseContainerDeclaration(
  PipelineRunner &Runner,
  const BackingContainerDeclaration &Declaration) const {
  auto It = KnownContainerTypes.find(Declaration.Type);
  if (It == KnownContainerTypes.end())
    return createStringError(inconvertibleErrorCode(),
                             "No known container with provided name");
  auto &Entry = It->second;
  Runner.addContainerFactory(Declaration.Name, Entry());

  return Error::success();
}

llvm::Expected<PipelineRunner>
PipelineLoader::load(const PipelineDeclaration &Declaration) const {
  PipelineRunner ToReturn;
  for (const auto &Container : Declaration.Containers)
    if (auto error = parseContainerDeclaration(ToReturn, Container); !!error)
      return move(error);

  for (const auto &Step : Declaration.Steps)
    if (auto error = parseStepDeclaration(ToReturn, Step); !!error)
      return move(error);

  ToReturn.addStep("End");

  return ToReturn;
}

llvm::Expected<PipelineRunner>
PipelineLoader::load(llvm::StringRef Pipeline) const {
  yaml::Input Input(Pipeline);

  PipelineDeclaration Declaration;
  Input >> Declaration;
  if (Input.error())
    return createStringError(inconvertibleErrorCode(),
                             "Could not parse pipeline");

  return load(Declaration);
}
