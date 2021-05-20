#pragma once

#include <memory>
#include <vector>
#include <string>

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/Model/TupleTree.h"

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/YAMLTraits.h"


namespace AutoEnforcer {
	struct BackingContainerDeclaration {
		std::string Name;
		std::string Type;
	};

	struct EnforcerInvocation {
		std::string Name;
		std::vector<std::string> UsedContainers;
	};

	struct StepDeclaration {
		std::string Name;
		std::vector<EnforcerInvocation> Enforcers;
	};

	struct PipelineDeclaration {
		std::vector<BackingContainerDeclaration> Containers;
		std::vector<StepDeclaration> Steps;
	};

	class PipelineLoader {
	  public:
	  llvm::Expected<PipelineRunner> load(const PipelineDeclaration&) const;
	  llvm::Expected<PipelineRunner> load(llvm::StringRef Pipeline) const;

	  template<typename ContainerType>
	  void addDefaultConstructibleContainer(llvm::StringRef Name) {
		  KnownContainerTypes.try_emplace(Name, [](){
						return std::make_unique<DefaultConstructibleBackingContainerFactory<ContainerType>>();
				  });
	  }

	  template<typename EnforcerType>
	  void addEnforcer(llvm::StringRef Name) {
		  KnownEnforcersTypes.try_emplace(Name, [](std::vector<std::string> ContainerNames){
				  return EnforcerWrapper::makeWrapper<EnforcerType>(std::move(ContainerNames));
				  });
	  }

	  private:
	  llvm::Error parseStepDeclaration(PipelineRunner& Runner, const StepDeclaration&) const;
	  llvm::Error parseInvocation(Step& Step, const EnforcerInvocation& Invocation) const;
	  llvm::Error parseContainerDeclaration(PipelineRunner& Runner, const BackingContainerDeclaration&) const;

	  llvm::StringMap<std::function<std::unique_ptr<BackingContainerRegistryEntry>()>> KnownContainerTypes;
	  llvm::StringMap<std::function<EnforcerWrapper(std::vector<std::string>)>> KnownEnforcersTypes;
	};
}

INTROSPECTION_NS(AutoEnforcer, BackingContainerDeclaration, Name, Type);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::BackingContainerDeclaration>
  : public TupleLikeMappingTraits<AutoEnforcer::BackingContainerDeclaration> {};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::BackingContainerDeclaration)

INTROSPECTION_NS(AutoEnforcer, EnforcerInvocation, Name, UsedContainers);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::EnforcerInvocation>
  : public TupleLikeMappingTraits<AutoEnforcer::EnforcerInvocation> {};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::EnforcerInvocation)

INTROSPECTION_NS(AutoEnforcer, StepDeclaration, Name, Enforcers);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::StepDeclaration>
  : public TupleLikeMappingTraits<AutoEnforcer::StepDeclaration> {};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::StepDeclaration)

INTROSPECTION_NS(AutoEnforcer, PipelineDeclaration, Containers, Steps);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::PipelineDeclaration>
  : public TupleLikeMappingTraits<AutoEnforcer::PipelineDeclaration> {};
