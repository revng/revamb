#pragma once
//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

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
  void registerDefaultConstructibleContainer(llvm::StringRef Name) {
    KnownContainerTypes.try_emplace(Name, []() {
      using Type = DefaultConstructibleBackingContainerFactory<ContainerType>;
      return std::make_unique<Type>();
    });
  }

  template<typename ContainerFactory, typename... Args>
  void registerContainerFactory(llvm::StringRef Name, Args &&... A) {
    KnownContainerTypes.try_emplace(Name, [&A...]() {
      return std::make_unique<ContainerFactory>(A...);
    });
  }

  template<typename LLVMEnforcerPass>
  void registerLLVMEnforcerPass(llvm::StringRef Name) {
    KnownLLVMEnforcerTypes.try_emplace(Name, []() {
      using Type = LLVMEnforcerImpl<LLVMEnforcerPass>;
      return std::make_unique<Type>(LLVMEnforcerPass());
    });
  }

  template<typename EnforcerType>
  void registerEnforcer(llvm::StringRef Name) {
    const auto LambdaToEmplace = [](std::vector<std::string> CName) {
      return EnforcerWrapper::makeWrapper<EnforcerType>(std::move(CName));
    };
    KnownEnforcersTypes.try_emplace(Name, LambdaToEmplace);
  }

  template<typename EnforcerType>
  void registerEnforcer(llvm::StringRef Name, const EnforcerType &Enforcer) {
    const auto LambdaToEmplace =
      [Enforcer](std::vector<std::string> ContainerNames) {
        return EnforcerWrapper(Enforcer, std::move(ContainerNames));
      };
    KnownEnforcersTypes.try_emplace(Name, LambdaToEmplace);
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

  using FType = std::function<std::unique_ptr<BackingContainerFactory>()>;
  llvm::StringMap<FType> KnownContainerTypes;
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
