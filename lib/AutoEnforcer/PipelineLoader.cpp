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
  for (const auto &Invocation : Declaration.Enforcers) {
    if (not isInvocationUsed(Invocation.EnabledWhen))
      continue;

    if (auto error = parseInvocation(Runner.back(), Invocation); !!error)
      return error;
  }

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
  auto Wrapper = EnforcerWrapper(move(ToInsert), Invocation.UsedContainers);
  Step.addEnforcer(move(Wrapper));

  return Error::success();
}

llvm::Error
PipelineLoader::parsePureLLVMPass(Step &Step,
                                  const EnforcerInvocation &Invocation) const {

  auto MaybeEnforcer = PureLLVMEnforcer::create(Invocation.Passess);
  if (!MaybeEnforcer)
    return MaybeEnforcer.takeError();

  auto Wrapper = EnforcerWrapper(move(*MaybeEnforcer),
                                 Invocation.UsedContainers);
  Step.addEnforcer(move(Wrapper));

  return Error::success();
}

llvm::Error
PipelineLoader::parseInvocation(Step &Step,
                                const EnforcerInvocation &Invocation) const {
  if (Invocation.Name == "LLVMEnforcer")
    return parseLLVMPass(Step, Invocation);

  if (Invocation.Name == "PureLLVMEnforcer")
    return parsePureLLVMPass(Step, Invocation);

  auto It = KnownEnforcersTypes.find(Invocation.Name);
  if (It == KnownEnforcersTypes.end())
    return createStringError(inconvertibleErrorCode(),
                             "while parsing inforcer invocation: No known "
                             "enforcer with name %s ",
                             Invocation.Name.c_str());
  auto &Entry = It->second;
  Step.addEnforcer(Entry(Invocation.UsedContainers));
  return Error::success();
}

using BCDecl = BackingContainerDeclaration;
Error PipelineLoader::parseContainerDeclaration(PipelineRunner &AE,
                                                const BCDecl &Dec) const {
  auto It = KnownContainerTypes.find(Dec.Type);
  if (It == KnownContainerTypes.end())
    return createStringError(inconvertibleErrorCode(),
                             "while parsing contaienr declaration: No known "
                             "container with name %s",
                             Dec.Type.c_str());
  auto &Entry = It->second;
  AE.registerContainerFactory(Dec.Name, Entry());

  return Error::success();
}

llvm::Expected<PipelineRunner>
PipelineLoader::load(const PipelineDeclaration &Declaration) const {
  PipelineRunner ToReturn;
  for (const auto &Container : Declaration.Containers)
    if (auto error = parseContainerDeclaration(ToReturn, Container); !!error)
      return move(error);

  for (const auto &Step : Declaration.Steps) {
    if (not isInvocationUsed(Step.EnabledWhen))
      continue;

    if (auto error = parseStepDeclaration(ToReturn, Step); !!error)
      return move(error);
  }

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

bool PipelineLoader::isInvocationUsed(const vector<string> &Invocation) const {
  if (Invocation.size() == 0)
    return true;

  const auto IsStringEnabled = [this](const std::string &Name) {
    if (not Name.starts_with("~"))
      return EnabledFlags.count(Name) != 0;

    string ActualName(Name.begin() + 1, Name.end());
    return EnabledFlags.count(ActualName) == 0;
  };
  return llvm::any_of(Invocation, IsStringEnabled);
}
