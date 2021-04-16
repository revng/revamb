/// \file IRHelpers.cpp
/// \brief Implementation of IR helper functions

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <fstream>

#include "llvm/Support/raw_os_ostream.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Support/IRHelpers.h"

// TODO: including GeneratedCodeBasicInfo.h is not very nice

using namespace llvm;

void dumpModule(const Module *M, const char *Path) {
  std::ofstream FileStream(Path);
  raw_os_ostream Stream(FileStream);
  M->print(Stream, nullptr, true);
}

GlobalVariable *buildString(Module *M, StringRef String, const Twine &Name) {
  LLVMContext &C = M->getContext();
  auto *Initializer = ConstantDataArray::getString(C, String, true);
  return new GlobalVariable(*M,
                            Initializer->getType(),
                            true,
                            GlobalVariable::InternalLinkage,
                            Initializer,
                            Name);
}

Constant *buildStringPtr(Module *M, StringRef String, const Twine &Name) {
  LLVMContext &C = M->getContext();
  Type *Int8PtrTy = Type::getInt8Ty(C)->getPointerTo();
  GlobalVariable *NewVariable = buildString(M, String, Name);
  return ConstantExpr::getBitCast(NewVariable, Int8PtrTy);
}

Constant *getUniqueString(Module *M,
                          StringRef Namespace,
                          StringRef String,
                          const Twine &Name) {
  LLVMContext &C = M->getContext();
  Type *Int8PtrTy = Type::getInt8Ty(C)->getPointerTo();
  NamedMDNode *StringsList = M->getOrInsertNamedMetadata(Namespace);

  for (MDNode *Operand : StringsList->operands()) {
    auto *T = cast<MDTuple>(Operand);
    revng_assert(T->getNumOperands() == 1);
    auto *CAM = cast<ConstantAsMetadata>(T->getOperand(0).get());
    auto *GV = cast<GlobalVariable>(CAM->getValue());
    revng_assert(GV->isConstant() and GV->hasInitializer());

    const Constant *Initializer = GV->getInitializer();
    StringRef Content = cast<ConstantDataArray>(Initializer)->getAsString();

    // Ignore the terminator
    if (Content.drop_back() == String)
      return ConstantExpr::getBitCast(GV, Int8PtrTy);
  }

  GlobalVariable *NewVariable = buildString(M, String, Name);
  auto *CAM = ConstantAsMetadata::get(NewVariable);
  StringsList->addOperand(MDTuple::get(C, { CAM }));
  return ConstantExpr::getBitCast(NewVariable, Int8PtrTy);
}

CallInst *reverseNewPCTraversal(Instruction *TheInstruction) {
  std::set<BasicBlock *> Visited;
  std::queue<BasicBlock::reverse_iterator> WorkList;

  // Initialize WorkList with an iterator pointing at the given instruction
  if (TheInstruction->getIterator() == TheInstruction->getParent()->begin())
    WorkList.push(--TheInstruction->getParent()->rend());
  else
    WorkList.push(++TheInstruction->getReverseIterator());

  // Process the worklist
  while (not WorkList.empty()) {
    auto I = WorkList.front();
    WorkList.pop();
    auto *BB = I->getParent();
    auto End = BB->rend();

    // Go through the instructions looking for calls to newpc
    for (; I != End; I++)
      if (CallInst *Marker = getCallTo(&*I, "newpc"))
        return Marker;

    // If we didn't find a newpc call yet, continue exploration backward
    // If one of the predecessors is the dispatcher, don't explore any further
    for (BasicBlock *Predecessor : predecessors(BB)) {
      using GCBI = GeneratedCodeBasicInfo;

      // Assert we didn't reach the almighty dispatcher
      revng_assert(GCBI::isPartOfRootDispatcher(Predecessor) == false);

      // Ignore already visited or empty BBs
      if (!Predecessor->empty() && Visited.find(Predecessor) == Visited.end()) {
        WorkList.push(Predecessor->rbegin());
        Visited.insert(Predecessor);
      }
    }
  }

  return nullptr;
}

std::pair<MetaAddress, uint64_t> getPC(Instruction *TheInstruction) {
  CallInst *NewPCCall = reverseNewPCTraversal(TheInstruction);

  // Couldn't find the current PC
  if (NewPCCall == nullptr)
    return { MetaAddress::invalid(), 0 };

  auto PC = MetaAddress::fromConstant(NewPCCall->getArgOperand(0));
  uint64_t Size = getLimitedValue(NewPCCall->getArgOperand(1));
  revng_assert(Size != 0);
  return { PC, Size };
}
