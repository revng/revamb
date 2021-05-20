#include "revng/AutoEnforcer/PipelineLoader.h"
#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "llvm/ADT/StringRef.h"
#include <optional>

using namespace AutoEnforcer;
using namespace std;
using namespace llvm;

llvm::Error PipelineLoader::parseStepDeclaration(PipelineRunner& Runner, const StepDeclaration& Declaration) const {
	Runner.addStep(Declaration.Name);
	for (const auto& Invocation : Declaration.Enforcers)
		if (auto error = parseInvocation(Runner.back(), Invocation); !!error)
			return error;

	return Error::success();
}

llvm::Error PipelineLoader::parseInvocation(Step& Step, const EnforcerInvocation& Invocation) const {
  auto It = KnownEnforcersTypes.find(Invocation.Name);
  if (It == KnownEnforcersTypes.end())
	  return createStringError(inconvertibleErrorCode(), "No known enforcer with provided name");
  auto& Entry = It->second; 
  Step.addEnforcer(Entry(Invocation.UsedContainers));
  return Error::success();
}

llvm::Error PipelineLoader::parseContainerDeclaration(PipelineRunner& Runner, 
													  const BackingContainerDeclaration& Declaration) const {
  auto It = KnownContainerTypes.find(Declaration.Type);
  if (It == KnownContainerTypes.end())
	  return createStringError(inconvertibleErrorCode(), "No known container with provided name");
  auto& Entry = It->second; 
  Runner.addContainerFactory(Declaration.Name, Entry());

  return Error::success();
}

llvm::Expected<PipelineRunner> PipelineLoader::load(const PipelineDeclaration& Declaration) const {
  PipelineRunner ToReturn;	
  for (const auto& Container : Declaration.Containers)
	 if (auto error = parseContainerDeclaration(ToReturn, Container); !!error)
		 return move(error);

  for (const auto& Step : Declaration.Steps)
	  if (auto error = parseStepDeclaration(ToReturn, Step); !!error)
		  return move(error);

  ToReturn.addStep("End");

  return ToReturn;
}


llvm::Expected<PipelineRunner> PipelineLoader::load(llvm::StringRef Pipeline) const {
	yaml::Input Input(Pipeline);

   PipelineDeclaration Declaration;
   Input >> Declaration;

   return load(Declaration);
}
