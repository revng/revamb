/// \file VariableManager.cpp
/// \brief This file handles the creation and management of global variables,
///        i.e. mainly parts of the CPU state

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <set>
#include <sstream>
#include <stack>
#include <string>

#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Type.h"
#include "llvm/Support/Casting.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/ValueMapper.h"

#include "revng/Lift/PTCDump.h"
#include "revng/Lift/PTCInterface.h"
#include "revng/Lift/VariableManager.h"
#include "revng/Support/Debug.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/revng.h"

#include "PTCDump.h"
#include "PTCInterface.h"
#include "VariableManager.h"

using namespace llvm;

// TODO: rename
cl::opt<bool> External("external",
                       cl::desc("set CSVs linkage to external, useful for "
                                "debugging purposes"),
                       cl::cat(MainCategory));
static cl::alias A1("E",
                    cl::desc("Alias for -external"),
                    cl::aliasopt(External),
                    cl::cat(MainCategory));

class OffsetValueStack {

private:
  using OffsetValuePair = std::pair<int64_t, Value *>;

public:
  void pushIfNew(int64_t Offset, Value *V) {
    OffsetValuePair Element = { Offset, V };
    if (!Seen.count(Element)) {
      Seen.insert(Element);
      Stack.push_back(Element);
    }
  }

  void push(int64_t Offset, Value *V) {
    OffsetValuePair Element = { Offset, V };
    Stack.push_back(Element);
  }

  bool empty() { return Stack.empty(); }

  std::pair<int64_t, Value *> pop() {
    auto Result = Stack.back();
    Stack.pop_back();
    return Result;
  }

  // TODO: this is on O(n)
  void cloneSisters(Value *Old, Value *New) {
    for (auto &OVP : Stack)
      if (OVP.second == Old)
        push(OVP.first, New);
  }

private:
  std::set<OffsetValuePair> Seen;
  std::vector<OffsetValuePair> Stack;
};

static std::pair<IntegerType *, unsigned>
getTypeAtOffset(const DataLayout *TheLayout, Type *VarType, intptr_t Offset) {
  static Logger<> Log("type-at-offset");

  unsigned Depth = 0;
  while (1) {
    switch (VarType->getTypeID()) {
    case llvm::Type::TypeID::PointerTyID:
      // BEWARE: here we return { nullptr, 0 } as an intended workaround for
      // a specific situation.
      //
      // We can't use assertions on pointers, as we do for all the other
      // unhandled types, because they will be inevitably triggered during the
      // execution. Indeed, all the other types are not present in QEMU
      // CPUState and we can safely assert it. This is not true for pointers
      // that are used in different places in QEMU CPUState.
      //
      // Given that we have ruled out assertions, we need to handle the
      // pointer case so that it keeps working. This function is expected to
      // return { nullptr, 0 } when the offset points to a memory location
      // associated to padding space. In principle, pointers are not padding
      // space, but the result of returning { nullptr, 0 } here is that load
      // and store operations treat pointers like padding. This means that
      // pointers cannot be read or written, and memcpy simply skips over them
      // leaving them alone.
      //
      // This behavior is intended, because a pointer into the CPUState could
      // be used to modify CPU registers indirectly, which is against all the
      // assumption of the analysis necessary for the translation, and also
      // against what really happens in a CPU, where CPU state cannot be
      // addressed.
      return { nullptr, 0 };

    case llvm::Type::TypeID::IntegerTyID:
      return { cast<IntegerType>(VarType), Offset };

    case llvm::Type::TypeID::ArrayTyID:
      VarType = VarType->getArrayElementType();
      Offset %= TheLayout->getTypeAllocSize(VarType);
      revng_log(Log,
                std::string(Depth++ * 2, ' ')
                  << " Is an Array. Offset in Element: " << Offset);
      break;

    case llvm::Type::TypeID::StructTyID: {
      StructType *TheStruct = cast<StructType>(VarType);
      const StructLayout *Layout = TheLayout->getStructLayout(TheStruct);
      unsigned FieldIndex = Layout->getElementContainingOffset(Offset);
      uint64_t FieldOffset = Layout->getElementOffset(FieldIndex);
      VarType = TheStruct->getTypeAtIndex(FieldIndex);
      intptr_t FieldEnd = FieldOffset + TheLayout->getTypeAllocSize(VarType);

      revng_log(Log,
                std::string(Depth++ * 2, ' ')
                  << " Offset: " << Offset
                  << " Struct Name: " << TheStruct->getName().str()
                  << " Field Index: " << FieldIndex << " Field offset: "
                  << FieldOffset << " Field end: " << FieldEnd);

      if (Offset >= FieldEnd)
        return { nullptr, 0 }; // It's padding

      Offset -= FieldOffset;
    } break;

    default:
      revng_abort("unexpected TypeID");
    }
  }
}

VariableManager::VariableManager(Module &M, Architecture &TargetArchitecture) :
  TheModule(M),
  AllocaBuilder(getContext(&M)),
  CPUStateType(nullptr),
  ModuleLayout(&TheModule.getDataLayout()),
  EnvOffset(0),
  Env(nullptr),
  TargetArchitecture(TargetArchitecture) {

  revng_assert(ptc.initialized_env != nullptr);

  IntegerType *IntPtrTy = AllocaBuilder.getIntPtrTy(*ModuleLayout);
  Env = cast<GlobalVariable>(TheModule.getOrInsertGlobal("env", IntPtrTy));
  Env->setInitializer(ConstantInt::getNullValue(IntPtrTy));

  using ElectionMap = std::map<StructType *, unsigned>;
  using ElectionMapElement = std::pair<StructType *const, unsigned>;
  ElectionMap EnvElection;

  std::set<StructType *> Structs;
  for (Function &HelperFunction : TheModule) {
    FunctionType *HelperType = HelperFunction.getFunctionType();
    Type *ReturnType = HelperType->getReturnType();
    if (ReturnType->isPointerTy())
      Structs.insert(dyn_cast<StructType>(ReturnType->getPointerElementType()));

    for (Type *Param : HelperType->params())
      if (Param->isPointerTy())
        Structs.insert(dyn_cast<StructType>(Param->getPointerElementType()));

    if (FunctionTags::QEMU.isTagOf(&HelperFunction)
        and HelperFunction.getName().startswith("helper_")
        and HelperFunction.getFunctionType()->getNumParams() > 1) {

      for (Type *Candidate : HelperType->params()) {
        Structs.insert(dyn_cast<StructType>(Candidate));
        if (Candidate->isPointerTy()) {
          auto *PointeeType = Candidate->getPointerElementType();
          auto *EnvType = dyn_cast<StructType>(PointeeType);
          // Ensure it is a struct and not a union
          if (EnvType != nullptr && EnvType->getNumElements() > 1) {

            auto It = EnvElection.find(EnvType);
            if (It != EnvElection.end())
              EnvElection[EnvType]++;
            else
              EnvElection[EnvType] = 1;
          }
        }
      }
    }
  }

  Structs.erase(nullptr);

  revng_assert(EnvElection.size() > 0);

  auto Compare = [](ElectionMapElement &It1, ElectionMapElement &It2) {
    return It1.second < It2.second;
  };
  auto Max = std::max_element(EnvElection.begin(), EnvElection.end(), Compare);
  CPUStateType = Max->first;

  // Look for structures containing CPUStateType as a member and promove them
  // to CPUStateType. Basically this is a flexible way to keep track of the *CPU
  // struct too (e.g. MIPSCPU).
  std::set<StructType *> Visited;
  bool Changed = true;
  Visited.insert(CPUStateType);
  while (Changed) {
    Changed = false;
    for (StructType *TheStruct : Structs) {
      if (Visited.find(TheStruct) != Visited.end())
        continue;

      auto Begin = TheStruct->element_begin();
      auto End = TheStruct->element_end();
      auto Found = std::find(Begin, End, CPUStateType);
      if (Found != End) {
        unsigned Index = Found - Begin;
        const StructLayout *Layout = nullptr;
        Layout = ModuleLayout->getStructLayout(TheStruct);
        EnvOffset += Layout->getElementOffset(Index);
        CPUStateType = TheStruct;
        Visited.insert(CPUStateType);
        Changed = true;
        break;
      }
    }
  }
}

Optional<StoreInst *>
VariableManager::storeToCPUStateOffset(IRBuilder<> &Builder,
                                       unsigned StoreSize,
                                       unsigned Offset,
                                       Value *ToStore) {
  Value *Target;
  unsigned Remaining;
  std::tie(Target, Remaining) = getByCPUStateOffsetInternal(Offset);

  if (Target == nullptr)
    return {};

  unsigned ShiftAmount = 0;
  if (TargetArchitecture.isLittleEndian())
    ShiftAmount = Remaining;
  else {
    // >> (Size1 - Size2) - Remaining;
    Type *PointeeTy = Target->getType()->getPointerElementType();
    unsigned GlobalSize = cast<IntegerType>(PointeeTy)->getBitWidth() / 8;
    revng_assert(GlobalSize != 0);
    ShiftAmount = (GlobalSize - StoreSize) - Remaining;
  }
  ShiftAmount *= 8;

  // Build blanking mask
  uint64_t BitMask = (StoreSize == 8 ? (uint64_t) -1 :
                                       ((uint64_t) 1 << StoreSize * 8) - 1);
  revng_assert(ShiftAmount != 64);
  BitMask <<= ShiftAmount;
  BitMask = ~BitMask;

  auto *InputStoreTy = cast<IntegerType>(Builder.getIntNTy(StoreSize * 8));
  auto *FieldTy = cast<IntegerType>(Target->getType()->getPointerElementType());
  unsigned FieldSize = FieldTy->getBitWidth() / 8;

  // Are we trying to store more than it fits?
  if (StoreSize > FieldSize) {
    // If we're storing more than it fits and the following memory is not
    // padding the store is not valid.
    if (getByCPUStateOffsetInternal(Offset + FieldSize).first != nullptr)
      return {};
  }

  // Truncate value to store
  auto *Truncated = Builder.CreateTrunc(ToStore, InputStoreTy);
  if (StoreSize > FieldSize)
    Truncated = Builder.CreateTrunc(Truncated, FieldTy);

  // Re-extend
  ToStore = Builder.CreateZExt(Truncated, FieldTy);

  if (BitMask != 0 and StoreSize != FieldSize) {
    // Load the value
    auto *LoadEnvField = Builder.CreateLoad(Target);

    auto *Blanked = Builder.CreateAnd(LoadEnvField, BitMask);

    // Shift value to store
    ToStore = Builder.CreateShl(ToStore, ShiftAmount);

    // Combine them
    ToStore = Builder.CreateOr(ToStore, Blanked);
  }

  return { Builder.CreateStore(ToStore, Target) };
}

Value *VariableManager::loadFromCPUStateOffset(IRBuilder<> &Builder,
                                               unsigned LoadSize,
                                               unsigned Offset) {
  Value *Target;
  unsigned Remaining;
  std::tie(Target, Remaining) = getByCPUStateOffsetInternal(Offset);

  if (Target == nullptr)
    return nullptr;

  // Load the whole field
  auto *LoadEnvField = Builder.CreateLoad(Target);

  // Extract the desired part
  // Shift right of the desired amount
  unsigned ShiftAmount = 0;
  if (TargetArchitecture.isLittleEndian()) {
    ShiftAmount = Remaining;
  } else {
    // >> (Size1 - Size2) - Remaining;
    auto *LoadedTy = cast<IntegerType>(LoadEnvField->getType());
    unsigned GlobalSize = LoadedTy->getBitWidth() / 8;
    revng_assert(GlobalSize != 0);
    ShiftAmount = (GlobalSize - LoadSize) - Remaining;
  }
  ShiftAmount *= 8;
  Value *Result = LoadEnvField;

  if (ShiftAmount != 0)
    Result = Builder.CreateLShr(Result, ShiftAmount);

  Type *LoadTy = Builder.getIntNTy(LoadSize * 8);

  // Are we trying to load more than its available in the field?
  if (auto FieldTy = dyn_cast<IntegerType>(Result->getType())) {
    unsigned FieldSize = FieldTy->getBitWidth() / 8;
    if (FieldSize < LoadSize) {
      // If after what we are loading ther is something that is not padding we
      // cannot load safely
      if (getByCPUStateOffsetInternal(Offset + FieldSize).first != nullptr)
        return nullptr;
      Result = Builder.CreateZExt(Result, LoadTy);
    }
  }

  // Truncate of the desired amount
  return Builder.CreateTrunc(Result, LoadTy);
}

bool VariableManager::memcpyAtEnvOffset(llvm::IRBuilder<> &Builder,
                                        llvm::CallInst *CallMemcpy,
                                        unsigned InitialEnvOffset,
                                        bool EnvIsSrc) {
  Function *Callee = getCallee(CallMemcpy);
  // We only support memcpys where the last parameter is constant
  revng_assert(Callee != nullptr
               and (Callee->getIntrinsicID() == Intrinsic::memcpy
                    and isa<ConstantInt>(CallMemcpy->getArgOperand(2))));

  Value *OtherOp = CallMemcpy->getArgOperand(EnvIsSrc ? 0 : 1);
  auto *MemcpySize = cast<Constant>(CallMemcpy->getArgOperand(2));
  Value *OtherBasePtr = Builder.CreatePtrToInt(OtherOp, Builder.getInt64Ty());

  uint64_t TotalSize = getZExtValue(MemcpySize, *ModuleLayout);
  uint64_t Offset = 0;

  bool OnlyPointersAndPadding = true;
  while (Offset < TotalSize) {
    GlobalVariable *EnvVar = getByEnvOffset(InitialEnvOffset + Offset).first;

    // Consider the case when there's simply nothing there (alignment space).
    if (EnvVar == nullptr) {
      // TODO: remove "false and", but after adding type based stuff
      if (false && EnvIsSrc) {
        ConstantInt *ZeroByte = Builder.getInt8(0);
        ConstantInt *OffsetInt = Builder.getInt64(Offset);
        Value *NewAddress = Builder.CreateAdd(OffsetInt, OtherBasePtr);
        Type *Int8PtrTy = Builder.getInt8Ty()->getPointerTo();
        Value *OtherPtr = Builder.CreateIntToPtr(NewAddress, Int8PtrTy);
        Builder.CreateStore(ZeroByte, OtherPtr);
        OnlyPointersAndPadding = false;
      }
      Offset++;
      continue;
    }
    OnlyPointersAndPadding = false;

    ConstantInt *OffsetInt = Builder.getInt64(Offset);
    Value *NewAddress = Builder.CreateAdd(OffsetInt, OtherBasePtr);
    Value *OtherPtr = Builder.CreateIntToPtr(NewAddress, EnvVar->getType());

    Value *Dst = EnvIsSrc ? OtherPtr : EnvVar;
    Value *Src = EnvIsSrc ? EnvVar : OtherPtr;
    Builder.CreateStore(Builder.CreateLoad(Src), Dst);

    Type *PointeeTy = EnvVar->getType()->getPointerElementType();
    Offset += ModuleLayout->getTypeAllocSize(PointeeTy);
  }

  if (OnlyPointersAndPadding)
    cast<Instruction>(OtherBasePtr)->eraseFromParent();

  return Offset == TotalSize;
}

void VariableManager::rebuildCSVList() {
  // Register the list of CSVs
  LLVMContext &Context = getContext(&TheModule);
  QuickMetadata QMD(Context);
  NamedMDNode *NamedMD = TheModule.getOrInsertNamedMetadata("revng.csv");
  std::vector<Metadata *> CSVsMD;
  for (auto &P : CPUStateGlobals)
    CSVsMD.push_back(QMD.get(P.second));
  NamedMD->clearOperands();
  NamedMD->addOperand(QMD.tuple(CSVsMD));
}

void VariableManager::finalize() {
  LLVMContext &Context = getContext(&TheModule);

  if (not External) {
    for (auto &P : CPUStateGlobals)
      P.second->setLinkage(GlobalValue::InternalLinkage);
    for (auto &P : OtherGlobals)
      P.second->setLinkage(GlobalValue::InternalLinkage);
  }

  IRBuilder<> Builder(Context);

  // Create the setRegister function
  auto *SetRegisterTy = FunctionType::get(Builder.getVoidTy(),
                                          { Builder.getInt32Ty(),
                                            Builder.getInt64Ty() },
                                          false);
  FunctionCallee SetRegisterC = TheModule.getOrInsertFunction("set_register",
                                                              SetRegisterTy);
  auto *SetRegister = cast<Function>(SetRegisterC.getCallee());
  SetRegister->setLinkage(GlobalValue::ExternalLinkage);

  // Collect arguments
  auto ArgIt = SetRegister->arg_begin();
  auto ArgEnd = SetRegister->arg_end();
  revng_assert(ArgIt != ArgEnd);
  Argument *RegisterID = &*ArgIt;
  ArgIt++;
  revng_assert(ArgIt != ArgEnd);
  Argument *NewValue = &*ArgIt;
  ArgIt++;
  revng_assert(ArgIt == ArgEnd);

  // Create main basic blocks
  using BasicBlock = BasicBlock;
  auto *EntryBB = BasicBlock::Create(Context, "", SetRegister);
  auto *DefaultBB = BasicBlock::Create(Context, "", SetRegister);
  auto *ReturnBB = BasicBlock::Create(Context, "", SetRegister);

  // Populate the default case of the switch
  Builder.SetInsertPoint(DefaultBB);
  Builder.CreateCall(TheModule.getFunction("abort"));
  Builder.CreateUnreachable();

  // Create the switch statement
  Builder.SetInsertPoint(EntryBB);
  auto *Switch = Builder.CreateSwitch(RegisterID,
                                      DefaultBB,
                                      CPUStateGlobals.size());
  for (auto &P : CPUStateGlobals) {
    Type *CSVTy = P.second->getType();
    auto *CSVIntTy = cast<IntegerType>(CSVTy->getPointerElementType());
    if (CSVIntTy->getBitWidth() <= 64) {
      // Set the value of the CSV
      auto *SetRegisterBB = BasicBlock::Create(Context, "", SetRegister);
      Builder.SetInsertPoint(SetRegisterBB);
      Builder.CreateStore(Builder.CreateTrunc(NewValue, CSVIntTy), P.second);
      Builder.CreateBr(ReturnBB);

      // Add the case to the switch
      Switch->addCase(Builder.getInt32(P.first), SetRegisterBB);
    }
  }

  // Finally, populate the return basic block
  Builder.SetInsertPoint(ReturnBB);
  Builder.CreateRetVoid();
}

// TODO: `newFunction` reflects the tcg terminology but in this context is
//       highly misleading
void VariableManager::newFunction(PTCInstructionList *Instructions) {
  LocalTemporaries.clear();
  this->Instructions = Instructions;
  newBasicBlock();
}

bool VariableManager::isEnv(Value *TheValue) {
  auto *Load = dyn_cast<LoadInst>(TheValue);
  if (Load != nullptr)
    return Load->getPointerOperand() == Env;

  return TheValue == Env;
}

static ConstantInt *fromBytes(IntegerType *Type, void *Data) {
  switch (Type->getBitWidth()) {
  case 8:
    return ConstantInt::get(Type, *(static_cast<uint8_t *>(Data)));
  case 16:
    return ConstantInt::get(Type, *(static_cast<uint16_t *>(Data)));
  case 32:
    return ConstantInt::get(Type, *(static_cast<uint32_t *>(Data)));
  case 64:
    return ConstantInt::get(Type, *(static_cast<uint64_t *>(Data)));
  }

  revng_unreachable("Unexpected type");
}

// TODO: document that it can return nullptr
GlobalVariable *
VariableManager::getByCPUStateOffset(intptr_t Offset, std::string Name) {
  GlobalVariable *Result = nullptr;
  unsigned Remaining;
  std::tie(Result, Remaining) = getByCPUStateOffsetInternal(Offset, Name);
  revng_assert(Remaining == 0);
  return Result;
}

std::pair<GlobalVariable *, unsigned>
VariableManager::getByCPUStateOffsetInternal(intptr_t Offset,
                                             std::string Name) {
  GlobalsMap::iterator it = CPUStateGlobals.find(Offset);
  static const char *UnknownCSVPref = "state_0x";
  if (it == CPUStateGlobals.end()
      || (Name.size() != 0
          && it->second->getName().startswith(UnknownCSVPref))) {
    Type *VariableType;
    unsigned Remaining;
    std::tie(VariableType,
             Remaining) = getTypeAtOffset(ModuleLayout, CPUStateType, Offset);

    // Unsupported type, let the caller handle the situation
    if (VariableType == nullptr)
      return { nullptr, 0 };

    // Check we're not trying to go inside an existing variable
    if (Remaining != 0) {
      GlobalsMap::iterator it = CPUStateGlobals.find(Offset - Remaining);
      if (it != CPUStateGlobals.end())
        return { it->second, Remaining };
    }

    if (Name.size() == 0) {
      std::stringstream NameStream;
      NameStream << UnknownCSVPref << std::hex << Offset;
      Name = NameStream.str();
    }

    // TODO: offset could be negative, we could segfault here
    auto *InitialValue = fromBytes(cast<IntegerType>(VariableType),
                                   ptc.initialized_env - EnvOffset + Offset);

    auto *NewVariable = new GlobalVariable(TheModule,
                                           VariableType,
                                           false,
                                           GlobalValue::ExternalLinkage,
                                           InitialValue,
                                           Name);
    revng_assert(NewVariable != nullptr);

    if (it != CPUStateGlobals.end()) {
      it->second->replaceAllUsesWith(NewVariable);
      it->second->eraseFromParent();
    }

    CPUStateGlobals[Offset] = NewVariable;

    rebuildCSVList();

    return { NewVariable, Remaining };
  } else {
    return { it->second, 0 };
  }
}

std::pair<bool, Value *>
VariableManager::getOrCreate(unsigned TemporaryId, bool Reading) {
  revng_assert(Instructions != nullptr);

  PTCTemp *Temporary = ptc_temp_get(Instructions, TemporaryId);
  Type *VariableType = Temporary->type == PTC_TYPE_I32 ?
                         AllocaBuilder.getInt32Ty() :
                         AllocaBuilder.getInt64Ty();

  if (ptc_temp_is_global(Instructions, TemporaryId)) {
    // Basically we use fixed_reg to detect "env"
    if (Temporary->fixed_reg == 0) {
      Value *Result = getByCPUStateOffset(EnvOffset + Temporary->mem_offset,
                                          Temporary->name);
      revng_assert(Result != nullptr);
      return { false, Result };
    } else {
      GlobalsMap::iterator it = OtherGlobals.find(TemporaryId);
      if (it != OtherGlobals.end()) {
        return { false, it->second };
      } else {
        // TODO: what do we have here, apart from env?
        auto InitialValue = ConstantInt::get(VariableType, 0);
        StringRef Name(Temporary->name);
        GlobalVariable *Result = nullptr;

        if (Name == "env") {
          revng_assert(Env != nullptr);
          Result = Env;
        } else {
          Result = new GlobalVariable(TheModule,
                                      VariableType,
                                      false,
                                      GlobalValue::CommonLinkage,
                                      InitialValue,
                                      Name);
        }

        OtherGlobals[TemporaryId] = Result;
        return { false, Result };
      }
    }
  } else if (Temporary->temp_local) {
    auto it = LocalTemporaries.find(TemporaryId);
    if (it != LocalTemporaries.end()) {
      return { false, it->second };
    } else {
      AllocaInst *NewTemporary = AllocaBuilder.CreateAlloca(VariableType);
      LocalTemporaries[TemporaryId] = NewTemporary;
      return { true, NewTemporary };
    }
  } else {
    auto it = Temporaries.find(TemporaryId);
    if (it != Temporaries.end()) {
      return { false, it->second };
    } else {
      // Can't read a temporary if it has never been written, we're probably
      // translating rubbish
      if (Reading)
        return { false, nullptr };

      AllocaInst *NewTemporary = AllocaBuilder.CreateAlloca(VariableType);
      Temporaries[TemporaryId] = NewTemporary;
      return { true, NewTemporary };
    }
  }
}

Value *VariableManager::computeEnvAddress(Type *TargetType,
                                          Instruction *InsertBefore,
                                          unsigned Offset) {

  auto *EnvPtrTy = cast<PointerType>(Env->getType());
  auto *PointeeTy = EnvPtrTy->getElementType();
  auto *LoadEnv = new LoadInst(PointeeTy, Env, "", InsertBefore);
  Type *EnvType = Env->getType()->getPointerElementType();
  Value *Integer = LoadEnv;
  if (Offset != 0)
    Integer = BinaryOperator::Create(Instruction::Add,
                                     LoadEnv,
                                     ConstantInt::get(EnvType, Offset),
                                     "",
                                     InsertBefore);
  return new IntToPtrInst(Integer, TargetType, "", InsertBefore);
}

Value *VariableManager::CPUStateToEnv(Value *CPUState,
                                      Type *TargetType,
                                      Instruction *InsertBefore) const {
  using CI = ConstantInt;

  Type *InputType = CPUState->getType()->getPointerElementType();
  revng_assert(InputType == CPUStateType
               or InputType == CPUStateType->getTypeAtIndex(0U));
  IRBuilder<> Builder(InsertBefore);
  Type *IntPtrTy = Builder.getIntPtrTy(*ModuleLayout);
  Value *CPUIntPtr = Builder.CreatePtrToInt(CPUState, IntPtrTy);
  Value *EnvIntPtr = Builder.CreateAdd(CPUIntPtr, CI::get(IntPtrTy, EnvOffset));
  return Builder.CreateIntToPtr(EnvIntPtr, TargetType);
}
