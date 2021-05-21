#pragma once

#include <memory>
#include <string>
#include <vector>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/YAMLTraits.h"

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/Model/TupleTree.h"

namespace AutoEnforcer {
struct BackingContainerDeclaration {
  std::string Name;
  std::string Type;
};

struct EnforcerInvocation {
  std::string Name;
  std::vector<std::string> UsedContainers;
  std::vector<std::string> Passess = {};
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
  llvm::Expected<PipelineRunner> load(const PipelineDeclaration &) const;
  llvm::Expected<PipelineRunner> load(llvm::StringRef Pipeline) const;

  template<typename ContainerType>
  void addDefaultConstructibleContainer(llvm::StringRef Name) {
    KnownContainerTypes.try_emplace(Name, []() {
      return std::make_unique<
        DefaultConstructibleBackingContainerFactory<ContainerType>>();
    });
  }

  template<typename ContainerFactory, typename... Args>
  void addContainerFactory(llvm::StringRef Name, Args &&... A) {
    KnownContainerTypes.try_emplace(Name, [&A...]() {
      return std::make_unique<ContainerFactory>(A...);
    });
  }

  template<typename LLVMEnforcerPass>
  void addLLVMEnforcerPass(llvm::StringRef Name) {
    KnownLLVMEnforcerTypes.try_emplace(Name, []() {
      using Type = LLVMEnforcerImpl<LLVMEnforcerPass>;
      return std::make_unique<Type>(
        std::forward<LLVMEnforcerPass>(LLVMEnforcerPass()));
    });
  }

  template<typename EnforcerType>
  void addEnforcer(llvm::StringRef Name) {
    KnownEnforcersTypes
      .try_emplace(Name, [](std::vector<std::string> ContainerNames) {
        return EnforcerWrapper::makeWrapper<EnforcerType>(
          std::move(ContainerNames));
      });
  }

private:
  llvm::Error
  parseStepDeclaration(PipelineRunner &Runner, const StepDeclaration &) const;
  llvm::Error
  parseInvocation(Step &Step, const EnforcerInvocation &Invocation) const;
  llvm::Error
  parseContainerDeclaration(PipelineRunner &Runner,
                            const BackingContainerDeclaration &) const;

  llvm::Error
  parseLLVMPass(Step &Step, const EnforcerInvocation &Invocation) const;

  llvm::StringMap<
    std::function<std::unique_ptr<BackingContainerRegistryEntry>()>>
    KnownContainerTypes;
  llvm::StringMap<std::function<EnforcerWrapper(std::vector<std::string>)>>
    KnownEnforcersTypes;
  llvm::StringMap<std::function<std::unique_ptr<LLVMEnforcerBaseImpl>()>>
    KnownLLVMEnforcerTypes;
};
} // namespace AutoEnforcer

INTROSPECTION_NS(AutoEnforcer, BackingContainerDeclaration, Name, Type);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::BackingContainerDeclaration>
  : public TupleLikeMappingTraits<AutoEnforcer::BackingContainerDeclaration> {};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::BackingContainerDeclaration)

template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::EnforcerInvocation> {
  static void mapping(IO &io, AutoEnforcer::EnforcerInvocation &info) {
    io.mapRequired("Name", info.Name);
    io.mapRequired("UsedContainers", info.UsedContainers);
    io.mapOptional("Passess", info.Passess);
  }
};

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
