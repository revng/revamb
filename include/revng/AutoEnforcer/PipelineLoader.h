#pragma once
//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <any>
#include <memory>
#include <string>
#include <vector>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/Support/YAMLTraits.h"

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/Model/TupleTree.h"
#include "revng/Support/Assert.h"

namespace AutoEnforcer {
struct BackingContainerDeclaration {
  std::string Name;
  std::string Type;
};

struct EnforcerInvocation {
  std::string Name;
  std::vector<std::string> UsedContainers;
  std::vector<std::string> Passess = {};
  std::vector<std::string> EnabledWhen = {};
};

struct StepDeclaration {
  std::string Name;
  std::vector<EnforcerInvocation> Enforcers;
  std::vector<std::string> EnabledWhen = {};
};

struct PipelineDeclaration {
  std::vector<BackingContainerDeclaration> Containers;
  std::vector<StepDeclaration> Steps;
};

template<typename Type>
char *getUniqueID() {
  static char ID;
  return &ID;
}

class TypeSafePtrWrapper {
public:
  template<typename T>
  TypeSafePtrWrapper(T &Object) : Ptr(&Object), ID(getUniqueID<T>()) {}

  template<typename T>
  bool isA() const {
    return ID == getUniqueID<T>();
  }

  template<typename T>
  T &getAs() const {
    revng_assert(isA<T>());
    return *static_cast<T *>(Ptr);
  }

private:
  void *Ptr;
  char *ID;
};

class PipelineLoader {
public:
  llvm::Expected<PipelineRunner> load(const PipelineDeclaration &) const;
  llvm::Expected<PipelineRunner> load(llvm::StringRef Pipeline) const;

  PipelineLoader(KindsRegisty KRegistry) : KindRegistry(std::move(KRegistry)) {}

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

  void registerEnabledFlags(auto &NamesRange) {
    for (const auto &Name : NamesRange)
      EnabledFlags.insert(Name);
  }

  template<typename T>
  llvm::Expected<T *> get(llvm::StringRef Name) const {
    auto Iter = Context.find(Name);
    if (Iter == Context.end())
      return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                     "pipeline loader context did not "
                                     "contained object %s",
                                     Name.str().c_str());
    if (not Iter->second.isA<T>())
      return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                     "pipeline loader was requested to cast %s "
                                     "to the wrong type",
                                     Name.str().c_str());

    return &Iter->second.getAs<T>();
  }

  void add(llvm::StringRef Name, auto &Object) {
    Context.try_emplace(Name, Object);
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

  llvm::Error
  parsePureLLVMPass(Step &Step, const EnforcerInvocation &Invocation) const;

  using FType = std::function<std::unique_ptr<BackingContainerFactory>()>;
  llvm::StringMap<FType> KnownContainerTypes;
  llvm::StringMap<std::function<EnforcerWrapper(std::vector<std::string>)>>
    KnownEnforcersTypes;
  llvm::StringMap<std::function<std::unique_ptr<LLVMEnforcerBaseImpl>()>>
    KnownLLVMEnforcerTypes;

  bool isInvocationUsed(const std::vector<std::string> &Names) const;

  std::set<std::string> EnabledFlags;

  llvm::StringMap<TypeSafePtrWrapper> Context;

  KindsRegisty KindRegistry;
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
    io.mapOptional("EnabledWhen", info.EnabledWhen);
  }
};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::EnforcerInvocation)

template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::StepDeclaration> {
  static void mapping(IO &io, AutoEnforcer::StepDeclaration &info) {
    io.mapRequired("Name", info.Name);
    io.mapRequired("Enforcers", info.Enforcers);
    io.mapOptional("EnabledWhen", info.EnabledWhen);
  }
};

LLVM_YAML_IS_SEQUENCE_VECTOR(AutoEnforcer::StepDeclaration)

INTROSPECTION_NS(AutoEnforcer, PipelineDeclaration, Containers, Steps);
template<>
struct llvm::yaml::MappingTraits<AutoEnforcer::PipelineDeclaration>
  : public TupleLikeMappingTraits<AutoEnforcer::PipelineDeclaration> {};
