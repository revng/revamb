#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/AssemblyAnnotationWriter.h"
#include "llvm/IR/LLVMContext.h"

/// \brief Annotate all instructions with self-referencing debug information
class SelfReferencingDbgAnnotationWriter
  : public llvm::AssemblyAnnotationWriter {
public:
  SelfReferencingDbgAnnotationWriter(llvm::LLVMContext &Context,
                                     llvm::AssemblyAnnotationWriter *InnerAAW) :
    Context(Context), InnerAAW(InnerAAW), DbgKind(Context.getMDKindID("dbg")) {}

  ~SelfReferencingDbgAnnotationWriter() override = default;

  virtual void emitFunctionAnnot(const llvm::Function *F,
                                 llvm::formatted_raw_ostream &Output) override {
    if (InnerAAW != nullptr)
      InnerAAW->emitFunctionAnnot(F, Output);
  }

  virtual void
  emitBasicBlockStartAnnot(const llvm::BasicBlock *BB,
                           llvm::formatted_raw_ostream &Output) override {
    if (InnerAAW != nullptr)
      InnerAAW->emitBasicBlockStartAnnot(BB, Output);
  }

  virtual void
  emitBasicBlockEndAnnot(const llvm::BasicBlock *BB,
                         llvm::formatted_raw_ostream &Output) override {
    if (InnerAAW != nullptr)
      InnerAAW->emitBasicBlockStartAnnot(BB, Output);
  }

  virtual void
  emitInstructionAnnot(const llvm::Instruction *O,
                       llvm::formatted_raw_ostream &Output) override;

  virtual void printInfoComment(const llvm::Value &V,
                                llvm::formatted_raw_ostream &Output) override {
    if (InnerAAW != nullptr)
      InnerAAW->printInfoComment(V, Output);
  }

private:
  llvm::LLVMContext &Context;
  llvm::AssemblyAnnotationWriter *InnerAAW;
  unsigned DbgKind;
};
