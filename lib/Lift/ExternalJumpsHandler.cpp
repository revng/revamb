/// \file externaljumpsHandler.cpp
/// \brief Inject code to support jumping in non-translated code and handling
///        the comeback.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <string>

#include "llvm/ADT/Triple.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/InlineAsm.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Lift/BinaryFile.h"
#include "revng/Lift/ExternalJumpsHandler.h"
#include "revng/Support/Debug.h"
#include "revng/Support/ProgramCounterHandler.h"

#include "BinaryFile.h"
#include "ExternalJumpsHandler.h"

using namespace llvm;
using std::string;

static string &
replace(string &Target, const StringRef Search, const StringRef Replace) {
  size_t Position = Target.find(Search.data());
  revng_assert(Position != string::npos);
  Target.replace(Position, Search.size(), Replace);
  return Target;
}

BasicBlock *ExternalJumpsHandler::createReturnFromExternal() {
  // Create return_from_call BasicBlock
  auto *ReturnFromExternal = BasicBlock::Create(Context,
                                                "return_from_external",
                                                &TheFunction);
  IRBuilder<> Builder(ReturnFromExternal);

  // Identify the global variables to be serialized
  Constant *SavedRegistersPtr = TheModule.getGlobalVariable("saved_registers");
  LoadInst *SavedRegisters = Builder.CreateLoad(SavedRegistersPtr);

  Value *GEP = Builder.CreateGEP(SavedRegisters,
                                 Builder.getInt32(Arch.pcMContextIndex()));
  LoadInst *PCAddress = Builder.CreateLoad(GEP);
  PCH->deserializePCFromSignalContext(Builder, PCAddress, SavedRegisters);

  // Deserialize the ABI registers
  for (const ABIRegister &Register : Arch.abiRegisters()) {
    GlobalVariable *CSV = TheModule.getGlobalVariable(Register.csvName());
    // Not all the registers have a corresponding CSV
    if (CSV != nullptr) {

      if (Register.inMContext()) {

        Constant *RegisterIndex = Builder.getInt32(Register.mcontextIndex());
        Value *GEP = Builder.CreateGEP(SavedRegisters, RegisterIndex);
        LoadInst *RegisterValue = Builder.CreateLoad(GEP);
        Builder.CreateStore(RegisterValue, CSV);

      } else {

        std::string AsmString = Arch.readRegisterAsm().str();
        replace(AsmString, "REGISTER", Register.name());
        std::stringstream ConstraintStringStream;
        ConstraintStringStream << "*m,~{},~{dirflag},~{fpsr},~{flags}";
        auto *FT = FunctionType::get(Type::getVoidTy(Context),
                                     { CSV->getType() },
                                     false);
        InlineAsm *Asm = InlineAsm::get(FT,
                                        AsmString,
                                        ConstraintStringStream.str(),
                                        true,
                                        InlineAsm::AsmDialect::AD_ATT);
        Builder.CreateCall(Asm, CSV);
      }
    }
  }

  Instruction *T = Builder.CreateBr(Dispatcher);
  setBlockType(T, BlockType::ExternalJumpsHandlerBlock);

  return ReturnFromExternal;
}

ExternalJumpsHandler::ExternalJumpsHandler(BinaryFile &TheBinary,
                                           BasicBlock *Dispatcher,
                                           Function &TheFunction,
                                           ProgramCounterHandler *PCH) :
  Context(getContext(&TheFunction)),
  QMD(Context),
  TheModule(*TheFunction.getParent()),
  TheFunction(TheFunction),
  TheBinary(TheBinary),
  Arch(TheBinary.architecture()),
  Dispatcher(Dispatcher),
  PCH(PCH) {
}

BasicBlock *ExternalJumpsHandler::createSerializeAndJumpOut() {
  // Create the serialize and branch Basic Block
  BasicBlock *Result = BasicBlock::Create(Context,
                                          "serialize_and_jump_out",
                                          &TheFunction);
  IRBuilder<> Builder(Result);
  auto *PC = PCH->loadJumpablePC(Builder);
  auto *JumpablePC = new GlobalVariable(TheModule,
                                        PC->getType(),
                                        false,
                                        GlobalValue::InternalLinkage,
                                        ConstantInt::get(PC->getType(), 0),
                                        "jumpablepc");
  Builder.CreateStore(PC, JumpablePC);

  // Serialize ABI CSVs
  for (const ABIRegister &Register : Arch.abiRegisters()) {
    GlobalVariable *CSV = TheModule.getGlobalVariable(Register.csvName());

    // Not all the registers have a corresponding CSV
    if (CSV == nullptr)
      continue;

    std::string AsmString = Arch.writeRegisterAsm().str();
    replace(AsmString, "REGISTER", Register.name());
    std::stringstream ConstraintStringStream;
    ConstraintStringStream << "*m,~{" << Register.name().data()
                           << "},~{dirflag},~{fpsr},~{flags}";
    auto *FT = FunctionType::get(Type::getVoidTy(Context),
                                 { CSV->getType() },
                                 false);
    InlineAsm *Asm = InlineAsm::get(FT,
                                    AsmString,
                                    ConstraintStringStream.str(),
                                    true,
                                    InlineAsm::AsmDialect::AD_ATT);
    Builder.CreateCall(Asm, CSV);
  }

  // Branch to the Program Counter address
  auto *FT = FunctionType::get(Type::getVoidTy(Context),
                               { JumpablePC->getType() },
                               false);
  InlineAsm *Asm = InlineAsm::get(FT,
                                  Arch.jumpAsm(),
                                  "*m,~{dirflag},~{fpsr},~{flags}",
                                  true,
                                  InlineAsm::AsmDialect::AD_ATT);
  Builder.CreateCall(Asm, JumpablePC);

  Instruction *T = Builder.CreateUnreachable();
  setBlockType(T, BlockType::ExternalJumpsHandlerBlock);

  return Result;
}

llvm::BasicBlock *ExternalJumpsHandler::createSetjmp(BasicBlock *FirstReturn,
                                                     BasicBlock *SecondReturn) {
  using CE = ConstantExpr;
  using CI = ConstantInt;

  BasicBlock *SetjmpBB = BasicBlock::Create(Context, "setjmp", &TheFunction);
  IRBuilder<> Builder(SetjmpBB);

  // Call setjmp
  llvm::Function *SetjmpFunction = TheModule.getFunction("setjmp");
  auto *SetJmpTy = SetjmpFunction->getType()->getPointerElementType();
  auto *JmpBuf = CE::getPointerCast(TheModule.getGlobalVariable("jmp_buffer"),
                                    SetJmpTy->getFunctionParamType(0));
  Value *SetjmpRes = Builder.CreateCall(SetjmpFunction, { JmpBuf });

  // Check if it's the first or second return
  auto *Zero = CI::get(cast<FunctionType>(SetJmpTy)->getReturnType(), 0);
  Value *BrCond = Builder.CreateICmpNE(SetjmpRes, Zero);

  Instruction *T = Builder.CreateCondBr(BrCond, SecondReturn, FirstReturn);
  setBlockType(T, BlockType::ExternalJumpsHandlerBlock);

  return SetjmpBB;
}

void ExternalJumpsHandler::buildExecutableSegmentsList() {
  IRBuilder<> Builder(Context);
  IntegerType *Int64 = Builder.getInt64Ty();
  SmallVector<Constant *, 10> ExecutableSegments;
  auto Int = [Int64](uint64_t V) { return ConstantInt::get(Int64, V); };
  for (auto &Segment : TheBinary.segments()) {
    if (Segment.IsExecutable) {
      ExecutableSegments.push_back(Int(Segment.StartVirtualAddress.address()));
      ExecutableSegments.push_back(Int(Segment.EndVirtualAddress.address()));
    }
  }

  auto *SegmentsType = ArrayType::get(Int64, ExecutableSegments.size());
  auto *SegmentsArray = ConstantArray::get(SegmentsType, ExecutableSegments);

  // Create the array (unnamed)
  auto *SegmentBoundaries = new GlobalVariable(TheModule,
                                               SegmentsArray->getType(),
                                               true,
                                               GlobalValue::InternalLinkage,
                                               SegmentsArray);

  // Create a pointer to the array (segment_boundaries) for support.c
  // consumption
  new GlobalVariable(TheModule,
                     Int64->getPointerTo(),
                     true,
                     GlobalValue::ExternalLinkage,
                     ConstantExpr::getPointerCast(SegmentBoundaries,
                                                  Int64->getPointerTo()),
                     "segment_boundaries");

  // Create a variable to hold the number of segments (segments_count)
  new GlobalVariable(TheModule,
                     Int64,
                     true,
                     GlobalValue::ExternalLinkage,
                     Int(ExecutableSegments.size() / 2),
                     "segments_count");
}

void ExternalJumpsHandler::createExternalJumpsHandler() {

  if (not Arch.isJumpOutSupported()) {
    buildExecutableSegmentsList();
    return;
  }

  BasicBlock *SerializeAndBranch = createSerializeAndJumpOut();
  BasicBlock *ReturnFromExternal = createReturnFromExternal();
  BasicBlock *SetjmpBB = createSetjmp(SerializeAndBranch, ReturnFromExternal);

  // Insert our BasicBlock as the default case of the dispatcher switch
  auto *Switch = cast<SwitchInst>(Dispatcher->getTerminator());
  BasicBlock *DispatcherFail = Switch->getDefaultDest();

  // Replace the default case of the dispatcher with the external jump handler.
  // In practice, perfrom a blind jump, unless the target is within the
  // executable segment of the current module.
  BasicBlock *ExternalJumpHandler = BasicBlock::Create(Context,
                                                       "dispatcher.external",
                                                       &TheFunction);

  DispatcherFail->replaceAllUsesWith(ExternalJumpHandler);

  {
    BasicBlock *IsExecutable = SetjmpBB;
    BasicBlock *IsNotExecutable = DispatcherFail;
    buildExecutableSegmentsList();

    Function *IsExecutableFunction = TheModule.getFunction("is_executable");
    IRBuilder<> Builder(ExternalJumpHandler);
    Value *PC = PCH->loadJumpablePC(Builder);
    Value *IsExecutableResult = Builder.CreateCall(IsExecutableFunction,
                                                   { PC });

    // If is_executable returns true go to default, otherwise setjmp
    Instruction *T = Builder.CreateCondBr(IsExecutableResult,
                                          IsNotExecutable,
                                          IsExecutable);
    setBlockType(T, BlockType::ExternalJumpsHandlerBlock);
  }
}
