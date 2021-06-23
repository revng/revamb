#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/StringRef.h"

namespace llvm {
class Module;
class AssemblyAnnotationWriter;
} // namespace llvm

void createSelfReferencingDebugInfo(llvm::Module *M,
                                    llvm::StringRef SourcePath,
                                    llvm::AssemblyAnnotationWriter *InnerAAW);
void createPTCDebugInfo(llvm::Module *M, llvm::StringRef SourcePath);
void createOriginalAssemblyDebugInfo(llvm::Module *M,
                                     llvm::StringRef SourcePath);
