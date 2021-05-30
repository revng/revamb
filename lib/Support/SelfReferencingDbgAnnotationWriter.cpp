/// \file SelfReferencingDbgAnnotationWriter.cpp
/// \brief This file handles debugging information generation.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Support/FormattedStream.h"

#include "revng/Support/FunctionTags.h"
#include "revng/Support/SelfReferencingDbgAnnotationWriter.h"

using namespace llvm;

using SRDAW = SelfReferencingDbgAnnotationWriter;
void SRDAW::emitInstructionAnnot(const Instruction *I,
                                 formatted_raw_ostream &Output) {
  if (InnerAAW != nullptr)
    InnerAAW->emitInstructionAnnot(I, Output);

  DISubprogram *Subprogram = I->getParent()->getParent()->getSubprogram();

  // Ignore whatever is outside the root and the isolated functions
  const Function *F = I->getParent()->getParent();
  if (Subprogram == nullptr or not isRootOrLifted(F))
    return;

  // Flushing is required to have correct line and column numbers
  Output.flush();

  auto *Location = DILocation::get(Context,
                                   Output.getLine() + 1,
                                   Output.getColumn(),
                                   Subprogram);

  // Sorry Bjarne
  auto *NonConstInstruction = const_cast<Instruction *>(I);
  NonConstInstruction->setMetadata(DbgKind, Location);
}
