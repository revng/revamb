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
#include "revng/Enforcers/Lift.h"
#include "revng/Enforcers/RevngEnforcers.h"
#include "revng/Lift/BinaryFile.h"
#include "revng/Lift/CodeGenerator.h"
#include "revng/Lift/PTCInterface.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/Debug.h"
#include "revng/Support/DebugHelper.h"
#include "revng/Support/OriginalAssemblyAnnotationWriter.h"
#include "revng/Support/ResourceFinder.h"
#include "revng/Support/Statistics.h"
#include "revng/Support/revng.h"

PTCInterface ptc = {}; ///< The interface with the PTC library.

using namespace llvm::cl;

using std::string;

// TODO: drop short aliases

static opt<string> InputPath(Positional, Required, desc("<input path>"));
static opt<string> OutputPath(Positional, Required, desc("<output path>"));

namespace DebugInfoType {

/// \brief Type of debug information to produce
enum Values {
  /// No debug information
  None,
  /// Produce a file containing the assembly code of the input binary
  OriginalAssembly,
  /// Produce the PTC as translated by libtinycode
  PTC,
  /// Prduce an LLVM IR with debug metadata referring to itself
  LLVMIR
};

} // namespace DebugInfoType

// Enable Debug Options to be specified on the command line
static auto X = values(clEnumValN(DebugInfoType::None,
                                  "none",
                                  "no debug information"),
                       clEnumValN(DebugInfoType::OriginalAssembly,
                                  "asm",
                                  "debug information referred to the "
                                  "assembly "
                                  "of the input file"),
                       clEnumValN(DebugInfoType::PTC,
                                  "ptc",
                                  "debug information referred to the "
                                  "Portable "
                                  "Tiny Code"),
                       clEnumValN(DebugInfoType::LLVMIR,
                                  "ll",
                                  "debug information referred to the LLVM "
                                  "IR"));
static opt<DebugInfoType::Values> DebugInfo("debug-info",
                                            desc("emit debug information"),
                                            X,
                                            cat(MainCategory),
                                            init(DebugInfoType::LLVMIR));

static alias A6("g",
                desc("Alias for -debug-info"),
                aliasopt(DebugInfo),
                cat(MainCategory));

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
  AE.registerContainerFactory<DefaultLLVMContainerFactory>(Lifted, C);
  AE.registerDefaultConstructibleFactory<BinaryContainer>(Binary);

  AE.addStep("first_step", bindEnforcer<LiftEnforcer>(Binary, Lifted));
  AE.addStep("end");

  // Load input file
  auto &BC = AE.getStartingContainer<BinaryContainer>(Binary);
  cantFail(BC.loadFromDisk(InputPath));

  // Establish goals
  BackingContainersStatus Targets;
  Targets.add(Lifted,
              AutoEnforcerTarget({ AutoEnforcerQuantifier("root") }, Root));

  // Run the pipeline
  auto Error = AE.run(Targets);
  revng_assert(not Error);

  // Save output file
  auto &Final = AE.getFinalContainer<DefaultLLVMContainer>(Lifted);
  // WIP
  auto *M = &Final.getModule();

  std::ofstream OutputStream(OutputPath);
  llvm::raw_os_ostream LLVMOutputStream(OutputStream);
  OriginalAssemblyAnnotationWriter OAAW(M->getContext());

  switch (DebugInfo) {
  case DebugInfoType::None:
    break;

  case DebugInfoType::OriginalAssembly:
    createOriginalAssemblyDebugInfo(M, OutputPath);
    break;

  case DebugInfoType::PTC:
    createPTCDebugInfo(M, OutputPath);
    break;

  case DebugInfoType::LLVMIR:
    createSelfReferencingDebugInfo(M, OutputPath, &OAAW);
    break;
  }

  M->print(LLVMOutputStream, &OAAW);
}
