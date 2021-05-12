#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <map>

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"

using CSVToAllocaMap = std::map<llvm::GlobalVariable *, llvm::AllocaInst *>;

/// \brief Helper class to generate calls that summarize pieces of codes
///        accessing CSVs
class SummaryCallsBuilder {
private:
  const CSVToAllocaMap &CSVMap;
  std::map<llvm::Type *, llvm::Function *> TemporaryFunctions;

public:
  SummaryCallsBuilder(const CSVToAllocaMap &CSVMap) : CSVMap(CSVMap) {}

public:
  llvm::CallInst *
  buildCall(llvm::IRBuilder<> &Builder,
            llvm::Type *ReturnType,
            llvm::ArrayRef<llvm::Value *> BaseArguments,
            llvm::ArrayRef<llvm::GlobalVariable *> ReadCSVs,
            llvm::ArrayRef<llvm::GlobalVariable *> WrittenCSVs) {
    using namespace llvm;

    Module *M = Builder.GetInsertBlock()->getParent()->getParent();

    std::vector<Value *> Arguments;
    std::copy(BaseArguments.begin(),
              BaseArguments.end(),
              std::back_inserter(Arguments));

    for (GlobalVariable *CSV : ReadCSVs)
      Arguments.push_back(Builder.CreateLoad(csvToAlloca(CSV)));

    CallInst *Result = Builder.CreateCall(getRandom(M, ReturnType), Arguments);

    // Put a `store getRandom()` targeting each written CSV
    for (GlobalVariable *Written : WrittenCSVs) {
      Type *PointeeTy = Written->getType()->getPointerElementType();
      Value *Random = Builder.CreateCall(getRandom(M, PointeeTy));
      Builder.CreateStore(Random, csvToAlloca(Written));
    }

    return Result;
  }

  void cleanup() {
    for (auto &P : TemporaryFunctions)
      P.second->eraseFromParent();

    TemporaryFunctions.clear();
  }

private:
  llvm::Value *csvToAlloca(llvm::GlobalVariable *CSV) const {
    auto It = CSVMap.find(CSV);
    if (It != CSVMap.end())
      return It->second;
    else
      return CSV;
  }

  llvm::Function *getRandom(llvm::Module *M, llvm::Type *ReturnType) {
    using namespace llvm;

    QuickMetadata QMD(M->getContext());

    auto It = TemporaryFunctions.find(ReturnType);
    if (It != TemporaryFunctions.end())
      return It->second;

    auto *Type = FunctionType::get(ReturnType, true);
    auto *F = Function::Create(Type, GlobalValue::ExternalLinkage, "", M);
    F->setOnlyAccessesInaccessibleMemory();
    TemporaryFunctions[ReturnType] = F;

    return F;
  }
};

/// \brief Replace calls to helper functions with read/writes to CSVs they use
///
/// This pass removes all the calls to helper functions replacing them with a
/// function call to `generic_helper` whose arguments (all variadic) are as
/// follows:
///
/// * a pointer to the original helper function;
/// * the original arguments of the call;
/// * the CSVs read by the helper according to CSAA;
///
/// After the replacement call, each CSV written by the helper (according to
/// CSAA) is clobbered with the result of a call to an opaque function.
///
/// This pass also marks the syscall ID argument of syscall helper functions
/// with `revng.syscallid`.
class DropHelperCallsPass : public llvm::PassInfoMixin<DropHelperCallsPass> {
private:
  SummaryCallsBuilder &SCB;
  llvm::Function *SyscallHelper;
  llvm::GlobalVariable *SyscallIDCSV;

public:
  DropHelperCallsPass(llvm::Function *SyscallHelper,
                      llvm::GlobalVariable *SyscallIDCSV,
                      SummaryCallsBuilder &SCB) :
    SCB(SCB), SyscallHelper(SyscallHelper), SyscallIDCSV(SyscallIDCSV) {}

  llvm::PreservedAnalyses
  run(llvm::Function &F, llvm::FunctionAnalysisManager &);

private:
};

inline llvm::PreservedAnalyses
DropHelperCallsPass::run(llvm::Function &F, llvm::FunctionAnalysisManager &) {
  using namespace llvm;

  LLVMContext &Context = getContext(&F);
  QuickMetadata QMD(Context);
  IRBuilder<> Builder(Context);
  std::vector<CallInst *> ToDelete;

  // TODO: iterating over users of helper functions would probably be faster
  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {

      if (auto *Call = getCallToHelper(&I)) {
        auto *Callee = cast<Function>(skipCasts(Call->getCalledOperand()));

        Builder.SetInsertPoint(Call);

        // Collect CSAA data
        auto CSVs = GeneratedCodeBasicInfo::getCSVUsedByHelperCall(Call);

        // Transform all the calls to helpers to calls to a function that
        // takes the following arguments:
        //
        // * A reference to the called function
        // * The original arguments
        // * All the registers read
        std::vector<Value *> BaseArguments = { Callee };
        auto CallArguments = make_range(Call->arg_begin(), Call->arg_end());
        for (Value *Argument : CallArguments)
          BaseArguments.push_back(Argument);

        Optional<uint32_t> SyscallIDArgumentIndex;
        for (GlobalVariable *CSV : CSVs.Read)
          if (Callee == SyscallHelper and CSV == SyscallIDCSV)
            SyscallIDArgumentIndex = BaseArguments.size();

        CallInst *NewCall = SCB.buildCall(Builder,
                                          I.getType(),
                                          BaseArguments,
                                          CSVs.Read,
                                          CSVs.Written);

        if (SyscallIDArgumentIndex) {
          NewCall->setMetadata("revng.syscallid",
                               QMD.tuple(*SyscallIDArgumentIndex));
        }

        // Drop the old function call
        I.replaceAllUsesWith(NewCall);
        ToDelete.push_back(Call);
      }
    }
  }

  for (CallInst *C : ToDelete)
    C->eraseFromParent();

  return PreservedAnalyses::none();
}
