/// \file Main.cpp

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdlib>

#include "llvm/ADT/StringMap.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/DynamicLibrary.h"
#include "llvm/Support/Error.h"
#include "llvm/Support/MemoryBuffer.h"

#include "revng/AutoEnforcer/AutoEnforcerLibraryRegistry.h"
#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainers.h"
#include "revng/AutoEnforcer/CopyEnforcer.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/AutoEnforcer/PipelineLoader.h"
#include "revng/Enforcers/RevngEnforcers.h"

using std::string;
using namespace llvm;
using namespace llvm::cl;
using namespace AutoEnforcer;

cl::OptionCategory AutoEnforcerCategory("Auto Enforcer Options", "");

static opt<string> InputPipeline(Positional,
                                 Required,
                                 desc("<Pipeline>"),
                                 cat(AutoEnforcerCategory));

static list<string> Targets(Positional,
                            Required,
                            desc("<Targets to produce>..."),
                            cat(AutoEnforcerCategory));

static list<string> ContainerOverrides("i",
                                       desc("Load the target file in the "
                                            "target container at the target "
                                            "step"),
                                       cat(AutoEnforcerCategory));

static opt<bool> DumpPipeline("d",
                              desc("Dump built pipeline and dont run"),
                              cat(AutoEnforcerCategory));

static list<string> StoresOverrides("o",
                                    desc("Store the target container at the "
                                         "target step in the target file"),
                                    cat(AutoEnforcerCategory));

static list<string> EnablingFlags("f",
                                  desc("list of pipeline enabling flags"),
                                  cat(AutoEnforcerCategory));

static opt<string> ExecutionFolder("p",
                                   desc("Folder from which all containers will "
                                        "be loaded before everything else and "
                                        "to which it will be store after "
                                        "everything else"),
                                   cat(AutoEnforcerCategory));

static list<string>
  loadLibraries("load", desc("libraries to open"), cat(AutoEnforcerCategory));

static alias A1("l",
                desc("Alias for --load"),
                aliasopt(loadLibraries),
                cat(AutoEnforcerCategory));

static ExitOnError exitOnError;

class LLVMAutoEnforcerLibraryRegistry : public AutoEnforcerLibraryRegistry {

public:
  void registerContainersAndEnforcers(PipelineLoader &Loader) override {
    auto MaybeLLVMContext = Loader.get<LLVMContext>("LLVMContext");
    if (!MaybeLLVMContext)
      return;

    using Type = DefaultLLVMContainerFactory;
    Loader.registerContainerFactory<Type>("LLVMContainer", **MaybeLLVMContext);
  }

  void registerKinds(llvm::StringMap<Kind *> &KindDictionary) override {
    KindDictionary["Root"] = &Root;
    KindDictionary["Isolated"] = &Isolated;
  }

  ~LLVMAutoEnforcerLibraryRegistry() override = default;
};

static LLVMAutoEnforcerLibraryRegistry Registry;

static auto getBuffer(StringRef Path) {
  return exitOnError(errorOrToExpected(MemoryBuffer::getFileOrSTDIN(Path)));
}

static PipelineRunner setUpAutoEnforcer(LLVMContext &Context) {
  PipelineLoader Loader;
  Loader.add("LLVMContext", Context);
  Loader.registerEnabledFlags(EnablingFlags);
  AutoEnforcerLibraryRegistry::registerAllContainersAndEnforcers(Loader);

  auto Pipeline = getBuffer(InputPipeline);
  auto AutoEnforcer = exitOnError(Loader.load(Pipeline->getBuffer()));

  if (not ExecutionFolder.empty())
    exitOnError(AutoEnforcer.load(ExecutionFolder));

  for (const auto &Override : ContainerOverrides) {
    auto Mapping = exitOnError(PipelineFileMapping::parse(Override));
    exitOnError(Mapping.load(AutoEnforcer));
  }

  return AutoEnforcer;
}

static void runAutoEnforcer(PipelineRunner &AutoEnforcer) {
  StringMap<Kind *> KindDictionary;
  AutoEnforcerLibraryRegistry::registerAllKinds(KindDictionary);

  BackingContainersStatus ToProduce;
  for (const auto &Target : Targets)
    exitOnError(parseAutoEnforcerTarget(ToProduce, Target, KindDictionary));

  exitOnError(AutoEnforcer.run(ToProduce));
}

static void tearDownAutoEnforcer(PipelineRunner &AutoEnforcer) {
  for (const auto &Override : StoresOverrides) {
    auto Mapping = exitOnError(PipelineFileMapping::parse(Override));

    exitOnError(Mapping.store(AutoEnforcer));
  }
  if (not ExecutionFolder.empty())
    exitOnError(AutoEnforcer.store(ExecutionFolder));
}

int main(int argc, const char *argv[]) {
  HideUnrelatedOptions(AutoEnforcerCategory);
  ParseCommandLineOptions(argc, argv);

  string Msg;
  for (const auto &Library : loadLibraries) {
    if (sys::DynamicLibrary::LoadLibraryPermanently(Library.c_str(), &Msg)) {
      dbg << Msg;
      return EXIT_FAILURE;
    }
  }

  LLVMContext Context;
  auto AutoEnforcer = setUpAutoEnforcer(Context);
  if (DumpPipeline) {
    AutoEnforcer.dump();
    return EXIT_SUCCESS;
  }
  runAutoEnforcer(AutoEnforcer);
  tearDownAutoEnforcer(AutoEnforcer);

  return EXIT_SUCCESS;
}
