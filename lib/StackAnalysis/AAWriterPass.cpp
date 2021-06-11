/// \file AAWriterPass.cpp
/// \brief Add aliasing scope information to the IR before load and store
/// accesses.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <vector>

#include "llvm/IR/Metadata.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/FormattedStream.h"

#include "revng/StackAnalysis/AAWriterPass.h"

using namespace llvm;

static cl::opt<std::string> AAWriterPath("aawriter",
                                         cl::value_desc("filename"),
                                         cl::desc("Dump to disk the outlined "
                                                  "functions with alias info "
                                                  "as comments."));

class AliasAnalysisAnnotatedWriter : public AssemblyAnnotationWriter {
  const bool StoresOnly;

public:
  AliasAnalysisAnnotatedWriter(bool StoresOnly) : StoresOnly(StoresOnly) {}

  void
  emitInstructionAnnot(const Instruction *I, formatted_raw_ostream &) override;
};

void AliasAnalysisAnnotatedWriter::emitInstructionAnnot(
  const Instruction *I,
  formatted_raw_ostream &OS) {
  QuickMetadata QMD(I->getContext());
  if (StoresOnly) {
    if (!isa<StoreInst>(I))
      return;
  } else {
    if (!isa<StoreInst>(I) && !isa<LoadInst>(I))
      return;
  }

  OS << "\n";
  OS.PadToColumn(2);
  OS << "; alias.scope: ";
  auto *AliasScopeMD = I->getMetadata(LLVMContext::MD_alias_scope);
  for (unsigned i = 0, e = AliasScopeMD->getNumOperands(); i != e; ++i) {
    auto *Tuple = cast<MDTuple>(AliasScopeMD->getOperand(i));
    OS << QMD.extract<StringRef>(Tuple, 0);
    if (i != (e - 1))
      OS << ", ";
  }

  OS << "\n";
  OS.PadToColumn(2);
  OS << "; noalias: ";
  auto *NoAliasScopeMD = I->getMetadata(LLVMContext::MD_noalias);
  for (unsigned i = 0, e = NoAliasScopeMD->getNumOperands(); i != e; ++i) {
    auto *Tuple = cast<MDTuple>(NoAliasScopeMD->getOperand(i));
    OS << QMD.extract<StringRef>(Tuple, 0);
    if (i != (e - 1))
      OS << ", ";
  }
  OS << "\n";
}

PreservedAnalyses AAWriterPass::run(Function &F, FunctionAnalysisManager &FAM) {
  if (AAWriterPath.empty())
    return PreservedAnalyses::all();

  std::unique_ptr<llvm::AssemblyAnnotationWriter> Annotator;
  Annotator.reset(new AliasAnalysisAnnotatedWriter(StoresOnly));

  std::error_code EC;
  raw_fd_ostream OS(AAWriterPath, EC, sys::fs::OF_Append | sys::fs::OF_Text);

  if (!EC) {
    formatted_raw_ostream FOS(OS);
    F.print(OS, Annotator.get(), true);
    OS.close();
  }

  return PreservedAnalyses::all();
}
