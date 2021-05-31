// WIP

#include <memory>

#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/AutoUpgrade.h"
#include "llvm/CodeGen/CommandFlags.inc"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Support/raw_os_ostream.h"

#include "revng/Enforcers/BinaryContainer.h"
#include "revng/Enforcers/CompileModule.h"

using namespace llvm;

void AutoEnforcer::CompileModuleEnforcer::run(DefaultLLVMContainer &TargetContainer,
                                              BinaryContainer &TargetBinary) {
  Module *M = &TargetContainer.getModule();

  // Get the target specific parser.
  std::string Error;
  Triple TheTriple(M->getTargetTriple());
  const Target *TheTarget = TargetRegistry::lookupTarget("",
                                                         TheTriple,
                                                         Error);
  revng_assert(TheTarget);

  TargetOptions Options = InitTargetOptionsFromCodeGenFlags();
  std::unique_ptr<TargetMachine> Target(TheTarget->createTargetMachine(TheTriple.getTriple(),
                                                                       "",
                                                                       "",
                                                                       Options,
                                                                       getRelocModel(),
                                                                       getCodeModel(),
                                                                       CodeGenOpt::Aggressive /* WIP */));


  // Add the target data from the target machine, if it exists, or the module.
  M->setDataLayout(Target->createDataLayout());

  // This needs to be done after setting datalayout since it calls verifier
  // to check debug info whereas verifier relies on correct datalayout.
  UpgradeDebugInfo(*M);

  LLVMTargetMachine &LLVMTM = static_cast<LLVMTargetMachine &>(*Target);
  MachineModuleInfoWrapperPass *MMIWP =
    new MachineModuleInfoWrapperPass(&LLVMTM);

  std::error_code EC;
  raw_fd_ostream OutputStream(TargetBinary.path(), EC);
  revng_assert(!EC);

  // Create pass manager
  legacy::PassManager PM;

  // Add an appropriate TargetLibraryInfo pass for the module's triple.
  TargetLibraryInfoImpl TLII(Triple(M->getTargetTriple()));
  PM.add(new TargetLibraryInfoWrapperPass(TLII));

  bool Result = Target->addPassesToEmitFile(PM,
                                            OutputStream,
                                            nullptr,
                                            CGFT_ObjectFile,
                                            true,
                                            MMIWP);
  revng_assert(Result);
  PM.run(*M);
}
