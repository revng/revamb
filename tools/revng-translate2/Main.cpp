/// \file Main.cpp
/// \brief This file takes care of handling command-line parameters and loading
/// the appropriate flavour of libtinycode-*.so

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/Optional.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/Object/Binary.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Signals.h"
#include "llvm/Support/raw_os_ostream.h"

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/Enforcers/BinaryContainer.h"
#include "revng/Enforcers/CompileModule.h"
#include "revng/Enforcers/Lift.h"
#include "revng/Enforcers/LinkSupport.h"
#include "revng/Enforcers/LinkForTranslation.h"
#include "revng/Enforcers/RevngEnforcers.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/Debug.h"
#include "revng/Support/DebugHelper.h"
#include "revng/Support/OriginalAssemblyAnnotationWriter.h"
#include "revng/Support/Statistics.h"
#include "revng/Support/revng.h"

using namespace llvm::cl;

opt<std::string> InputPath(Positional, Required, desc("<input path>"));
opt<std::string> OutputPath(Positional, Required, desc("<output path>"));

int main(int argc, const char *argv[]) {
  // Enable LLVM stack trace
  llvm::sys::PrintStackTraceOnErrorSignal(argv[0]);

  HideUnrelatedOptions({ &MainCategory });
  ParseCommandLineOptions(argc, argv);
  installStatistics();

  llvm::LLVMContext C;

  using namespace AutoEnforcer;
  PipelineRunner AE;
  std::string Lifted = "lifted.ll";
  std::string Binary = "binary";
  std::string ObjectFile = "object.o";
  std::string TranslatedBinary = "translated-binary";
  AE.registerContainerFactory<DefaultLLVMContainerFactory>(Lifted, C);
  AE.registerDefaultConstructibleFactory<BinaryContainer>(Binary);
  AE.registerDefaultConstructibleFactory<BinaryContainer>(ObjectFile);
  AE.registerDefaultConstructibleFactory<BinaryContainer>(TranslatedBinary);

  AE.addStep("first_step",
             bindEnforcer<LiftEnforcer>(Binary, Lifted),
             bindEnforcer<LinkSupportEnforcer>(Lifted),
             bindEnforcer<CompileModuleEnforcer>(Lifted, ObjectFile),
             bindEnforcer<LinkForTranslationEnforcer>(Lifted,
                                                      Binary,
                                                      ObjectFile,
                                                      TranslatedBinary));
  AE.addStep("end");

  // Load input file
  auto &BC = AE.getStartingContainer<BinaryContainer>(Binary);
  cantFail(BC.loadFromDisk(InputPath));

  // Establish goals
  BackingContainersStatus Targets;
  Targets.add(Lifted,
              AutoEnforcerTarget({ AutoEnforcerQuantifier("root") }, Translated));

  // Run the pipeline
  auto Error = AE.run(Targets);
  revng_assert(not Error);

  // Save output file
  auto &Final = AE.getFinalContainer<BinaryContainer>(TranslatedBinary);
  cantFail(BC.storeToDisk(Final.path()));
}
