/// \file DebugHelper.cpp
/// \brief This file handles debugging information generation.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <fstream>

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DIBuilder.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/raw_os_ostream.h"

#include "revng/Support/Assert.h"
#include "revng/Support/DebugHelper.h"
#include "revng/Support/FunctionTags.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/SelfReferencingDbgAnnotationWriter.h"

using namespace llvm;

// WIP: rename DebugHelper.h

/// Add a module flag, if not already present, using name and value provided.
/// Used for creating the Dwarf compliant debug info.
static void addModuleFlag(Module *M, StringRef Flag, uint32_t Value) {
  if (M->getModuleFlag(Flag) == nullptr) {
    M->addModuleFlag(Module::Warning, Flag, Value);
  }
}

static void createModuleDebugInfo(Module *M, llvm::StringRef SourcePath) {
  DIBuilder Builder(*M);
  auto File = Builder.createFile(SourcePath, "");
  DICompileUnit *CompileUnit = nullptr;
  CompileUnit = Builder.createCompileUnit(dwarf::DW_LANG_C,
                                          File,
                                          "revng",
                                          false,
                                          "",
                                          0 /* Runtime version */);

  // Add the current debug info version into the module after checking if it
  // is already present.
  addModuleFlag(M, "Debug Info Version", DEBUG_METADATA_VERSION);
  addModuleFlag(M, "Dwarf Version", 4);

  for (Function &F : M->functions()) {
    if (isRootOrLifted(&F)) {
      DISubroutineType *EmptyType = nullptr;
      DITypeRefArray EmptyArrayType = Builder.getOrCreateTypeArray({});
      EmptyType = Builder.createSubroutineType(EmptyArrayType);

      revng_assert(CompileUnit != nullptr);
      DISubprogram *Subprogram = nullptr;
      auto SPFlags = DISubprogram::toSPFlags(false, /* isLocalToUnit */
                                             true, /* isDefinition */
                                             false /* isOptimized */);
      Subprogram = Builder.createFunction(CompileUnit->getFile(), // Scope
                                          F.getName(), // Name
                                          StringRef(), // LinkageName
                                          CompileUnit->getFile(), // File
                                          1, // LineNo
                                          EmptyType, // Ty (subroutine type)
                                          1, // ScopeLine
                                          DINode::FlagPrototyped, // Flags
                                          SPFlags);
      F.setSubprogram(Subprogram);
    }
  }

  Builder.finalize();
}

void createSelfReferencingDebugInfo(Module *M,
                                    StringRef SourcePath,
                                    AssemblyAnnotationWriter *InnerAAW) {
  createModuleDebugInfo(M, SourcePath);

  SelfReferencingDbgAnnotationWriter Annotator(M->getContext(), InnerAAW);

  raw_null_ostream NullStream;
  M->print(NullStream, &Annotator);
}

static void createDebugInfoFromMetadata(Module *M,
                                        StringRef SourcePath,
                                        StringRef MetadataName) {
  createModuleDebugInfo(M, SourcePath);

  std::ofstream SourceOutputStream(SourcePath);
  raw_os_ostream SourceOutput(SourceOutputStream);

  // Generate the source file and the debugging information in tandem
  unsigned LineIndex = 1;
  unsigned MetadataKind = M->getContext().getMDKindID(MetadataName);
  unsigned DbgKind = M->getContext().getMDKindID("dbg");

  StringRef Last;
  for (Function &F : M->functions()) {

    if (not isRootOrLifted(&F))
      continue;

    if (DISubprogram *CurrentSubprogram = F.getSubprogram()) {
      for (BasicBlock &Block : F) {
        for (Instruction &I : Block) {
          StringRef Body = getText(&I, MetadataKind);

          if (Body.size() != 0 && Last != Body) {
            Last = Body;
            SourceOutput << Body.data();

            auto *Location = DILocation::get(M->getContext(),
                                             LineIndex,
                                             0,
                                             CurrentSubprogram);
            I.setMetadata(DbgKind, Location);
            LineIndex += std::count(Body.begin(), Body.end(), '\n');
          }
        }
      }
    }
  }
}

void createPTCDebugInfo(Module *M, StringRef SourcePath) {
  createDebugInfoFromMetadata(M, SourcePath, "pi");
}

void createOriginalAssemblyDebugInfo(Module *M, StringRef SourcePath) {
  createDebugInfoFromMetadata(M, SourcePath, "oi");
}
