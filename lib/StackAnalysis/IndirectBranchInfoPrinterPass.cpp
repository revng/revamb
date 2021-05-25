/// \file IndirectBranchInfoPrinterPass.cpp
/// \brief Serialize the results of the StackAnalysis on disk.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/Support/CommandLine.h"

#include "revng/StackAnalysis/IndirectBranchInfoPrinterPass.h"
#include "revng/Support/IRHelpers.h"

using namespace llvm;

static cl::opt<std::string> IndirectBranchInfoPath("indirect-branch-info-"
                                                   "summary",
                                                   cl::value_desc("filename"),
                                                   cl::desc("Write the results "
                                                            "of SA2 on disk."));

void IndirectBranchInfoPrinterPass::serialize(
  raw_ostream &OS,
  CallBase *Call,
  SmallVectorImpl<GlobalVariable *> &ABIRegs) {
  auto Text = MemoryBuffer::getFileAsStream(IndirectBranchInfoPath);
  if (!Text.getError()) {
    if (!Text->get()->getBuffer().startswith("name")) {
      OS << "name,ra,fso,address";
      for (const auto &Reg : ABIRegs)
        OS << "," << Reg->getName();
      OS << "\n";
    }
  }

  OS << Call->getParent()->getParent()->getName();
  for (unsigned I = 0; I < Call->getNumArgOperands(); ++I) {
    if (isa<ConstantInt>(Call->getArgOperand(I))) {
      OS << "," << cast<ConstantInt>(Call->getArgOperand(I))->getSExtValue();
    } else if (isa<StructType>(Call->getArgOperand(I)->getType())) {
      auto PC = MetaAddress::fromConstant(Call->getArgOperand(I));
      OS << "," << PC.address();
    } else {
      OS << ",unknown";
    }
  }
  OS << "\n";
}

PreservedAnalyses
IndirectBranchInfoPrinterPass::run(Function &F, FunctionAnalysisManager &FAM) {
  if (IndirectBranchInfoPath.empty())
    return PreservedAnalyses::all();

  // Get the result of the GCBI analysis
  auto &M = *F.getParent();
  const auto &MAMProxy = FAM.getResult<ModuleAnalysisManagerFunctionProxy>(F);
  GCBI = MAMProxy.getCachedResult<GeneratedCodeBasicInfoAnalysis>(M);
  if (!GCBI)
    GCBI = &(FAM.getResult<GeneratedCodeBasicInfoAnalysis>(F));
  revng_assert(GCBI != nullptr);

  SmallVector<GlobalVariable *, 16> ABIRegisters;
  for (auto *CSV : GCBI->abiRegisters())
    if (CSV && !(GCBI->isSPReg(CSV)))
      ABIRegisters.emplace_back(CSV);

  std::error_code EC;
  raw_fd_ostream OS(IndirectBranchInfoPath,
                    EC,
                    sys::fs::OF_Append | sys::fs::OF_Text);

  if (!EC) {
    for (auto *Call : callers(M.getFunction("indirect_branch_info")))
      if (Call->getParent()->getParent() == &F)
        serialize(OS, Call, ABIRegisters);
    OS.close();
  }

  return PreservedAnalyses::all();
}
