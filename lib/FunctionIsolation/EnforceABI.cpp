/// \file EnforceABI.cpp
/// \brief Promotes global variables CSV to function arguments or local
///        variables, according to the ABI analysis.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalAlias.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Support/raw_os_ostream.h"

#include "revng/ADT/LazySmallBitVector.h"
#include "revng/ADT/SmallMap.h"
#include "revng/FunctionIsolation/EnforceABI.h"
#include "revng/StackAnalysis/FunctionsSummary.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/OpaqueFunctionsPool.h"

using namespace llvm;

using StackAnalysis::FunctionCallRegisterArgument;
using StackAnalysis::FunctionCallReturnValue;
using StackAnalysis::FunctionRegisterArgument;
using StackAnalysis::FunctionReturnValue;
using StackAnalysis::FunctionsSummary;

using CallSiteDescription = FunctionsSummary::CallSiteDescription;
using FunctionDescription = FunctionsSummary::FunctionDescription;
using FCRD = FunctionsSummary::FunctionCallRegisterDescription;
using FunctionCallRegisterDescription = FCRD;
using FRD = FunctionsSummary::FunctionRegisterDescription;
using FunctionRegisterDescription = FRD;

char EnforceABI::ID = 0;
using Register = RegisterPass<EnforceABI>;
static Register X("enforce-abi", "Enforce ABI Pass", true, true);

static Logger<> EnforceABILog("enforce-abi");
static cl::opt<bool> DisableSafetyChecks("disable-enforce-abi-safety-checks",
                                         cl::desc("Disable safety checks in "
                                                  " ABI enforcing"),
                                         cl::cat(MainCategory),
                                         cl::init(false));

static bool shouldEmit(FunctionRegisterArgument V) {
  switch (V.value()) {
  case FunctionRegisterArgument::Maybe:
  case FunctionRegisterArgument::Yes:
    return true;
  default:
    return false;
  }
}

static bool shouldEmit(FunctionCallRegisterArgument V) {
  switch (V.value()) {
  case FunctionCallRegisterArgument::Yes:
  case FunctionCallRegisterArgument::Dead:
    return true;
  default:
    return false;
  }
}

static bool shouldEmit(FunctionReturnValue V) {
  switch (V.value()) {
  case FunctionReturnValue::Maybe:
  case FunctionReturnValue::YesOrDead:
    return true;
  default:
    return false;
  }
}

static bool shouldEmit(FunctionCallReturnValue V) {
  switch (V.value()) {
  case FunctionCallReturnValue::YesOrDead:
  case FunctionCallReturnValue::Yes:
  case FunctionCallReturnValue::Dead:
    return true;
  default:
    return false;
  }
}

class HelperCallSite {
public:
  using CSVToIndexMap = std::map<GlobalVariable *, unsigned>;
  HelperCallSite(const std::vector<GlobalVariable *> &ReadCSVs,
                 const std::vector<GlobalVariable *> &WrittenCSVs,
                 const CSVToIndexMap &CSVToIndex,
                 Function *Helper) :
    Helper(Helper) {
    revng_assert(Helper != nullptr);

    populateBitmap(CSVToIndex, ReadCSVs, Read);
    populateBitmap(CSVToIndex, WrittenCSVs, Written);
  }

public:
  bool operator<(const HelperCallSite &Other) const {
    auto ThisTie = std::tie(Helper, Read, Written);
    auto OtherTie = std::tie(Other.Helper, Other.Read, Other.Written);
    return ThisTie < OtherTie;
  }

  bool operator==(const HelperCallSite &Other) const {
    auto ThisTie = std::tie(Helper, Read, Written);
    auto OtherTie = std::tie(Other.Helper, Other.Read, Other.Written);
    return ThisTie == OtherTie;
  }

  Function *helper() const { return Helper; }

private:
  static void populateBitmap(const CSVToIndexMap &CSVToIndex,
                             const std::vector<GlobalVariable *> &Source,
                             LazySmallBitVector &Target) {
    for (GlobalVariable *CSV : Source) {
      Target.set(CSVToIndex.at(CSV));
    }
  }

private:
  Function *Helper;
  LazySmallBitVector Read;
  LazySmallBitVector Written;
};

class EnforceABIImpl {
public:
  EnforceABIImpl(Module &M, const GeneratedCodeBasicInfo &GCBI) :
    M(M),
    GCBI(GCBI),
    FunctionDispatcher(M.getFunction("function_dispatcher")),
    Context(M.getContext()),
    IndirectPlaceholderPool(&M, false) {}

  void run();

private:
  FunctionsSummary::FunctionDescription handleFunction(Function &F);

  void handleRegularFunctionCall(CallInst *Call);
  void handleHelperFunctionCall(CallInst *Call);
  void generateCall(IRBuilder<> &Builder,
                    Function *Callee,
                    FunctionsSummary::CallSiteDescription &CallSite,
                    bool IsPartOfFunctionDispatcher);
  void replaceCSVsWithAlloca();
  void replaceCallWithInvoke();
  void CSVsSerialization(Function *F, BasicBlock *RestoreBB);
  void installUnwindBasicBlock(Function *F);
  void handleRoot();

private:
  Module &M;
  const GeneratedCodeBasicInfo &GCBI;
  std::map<Function *, FunctionsSummary::FunctionDescription> FunctionsMap;
  std::map<Function *, Function *> OldToNew;
  Function *FunctionDispatcher;
  Function *OpaquePC;
  DenseMap<std::pair<GlobalVariable *, Function *>, AllocaInst *> CSVMap;
  DenseMap<Function *, BasicBlock *> UnwindEdgesMap;
  Function *RaiseException;
  GlobalVariable *RestoreCSVBitmask;
  LLVMContext &Context;
  std::map<HelperCallSite, Function *> HelperCallSites;
  std::map<GlobalVariable *, unsigned> CSVToIndex;
  OpaqueFunctionsPool<FunctionType *> IndirectPlaceholderPool;
};

bool EnforceABI::runOnModule(Module &M) {
  auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();

  EnforceABIImpl Impl(M, GCBI);
  Impl.run();
  return false;
}

// TODO: assign alias information
static Function *
createHelperWrapper(Function *Helper,
                    const std::vector<GlobalVariable *> &Read,
                    const std::vector<GlobalVariable *> &Written) {
  auto *PointeeTy = Helper->getType()->getPointerElementType();
  auto *HelperType = cast<FunctionType>(PointeeTy);

  //
  // Create new argument list
  //
  SmallVector<Type *, 16> NewArguments;

  // Initialize with base arguments
  std::copy(HelperType->param_begin(),
            HelperType->param_end(),
            std::back_inserter(NewArguments));

  // Add type of read registers
  for (GlobalVariable *CSV : Read)
    NewArguments.push_back(CSV->getType()->getPointerElementType());

  //
  // Create return type
  //

  // If the helpers does not write any register, reuse the original
  // return type
  Type *OriginalReturnType = HelperType->getReturnType();
  Type *NewReturnType = OriginalReturnType;

  bool HasOutputCSVs = Written.size() != 0;
  bool OriginalWasVoid = OriginalReturnType->isVoidTy();
  if (HasOutputCSVs) {
    SmallVector<Type *, 16> ReturnTypes;

    // If the original return type was not void, put it as first field
    // in the return type struct
    if (not OriginalWasVoid) {
      ReturnTypes.push_back(OriginalReturnType);
    }

    for (GlobalVariable *CSV : Written)
      ReturnTypes.push_back(CSV->getType()->getPointerElementType());

    NewReturnType = StructType::create(ReturnTypes);
  }

  //
  // Create new helper wrapper function
  //
  auto *NewHelperType = FunctionType::get(NewReturnType, NewArguments, false);
  auto *HelperWrapper = Function::Create(NewHelperType,
                                         Helper->getLinkage(),
                                         Twine(Helper->getName()) + "_wrapper",
                                         Helper->getParent());

  auto *Entry = BasicBlock::Create(getContext(Helper), "", HelperWrapper);

  //
  // Populate the helper wrapper function
  //
  IRBuilder<> Builder(Entry);

  // Serialize read CSV
  auto It = HelperWrapper->arg_begin();
  for (unsigned I = 0; I < HelperType->getNumParams(); I++, It++) {
    // Do nothing
    revng_assert(It != HelperWrapper->arg_end());
  }

  for (GlobalVariable *CSV : Read) {
    revng_assert(It != HelperWrapper->arg_end());
    Builder.CreateStore(&*It, CSV);
    It++;
  }
  revng_assert(It == HelperWrapper->arg_end());

  // Prepare the arguments
  SmallVector<Value *, 16> HelperArguments;
  It = HelperWrapper->arg_begin();
  for (unsigned I = 0; I < HelperType->getNumParams(); I++, It++) {
    revng_assert(It != HelperWrapper->arg_end());
    HelperArguments.push_back(&*It);
  }

  // Create the function call
  auto *HelperResult = Builder.CreateCall(Helper, HelperArguments);

  // Deserialize and return the appropriate values
  if (HasOutputCSVs) {
    SmallVector<Value *, 16> ReturnValues;

    if (not OriginalWasVoid)
      ReturnValues.push_back(HelperResult);

    for (GlobalVariable *CSV : Written)
      ReturnValues.push_back(Builder.CreateLoad(CSV));

    Builder.CreateAggregateRet(ReturnValues.data(), ReturnValues.size());

  } else if (OriginalWasVoid) {
    Builder.CreateRetVoid();
  } else {
    Builder.CreateRet(HelperResult);
  }

  return HelperWrapper;
}

void EnforceABIImpl::run() {

  // Declare an opaque function used later to obtain a value to store in the
  // local %pc alloca, so that we don't incur in error when removing the bad
  // return pc checks.
  if (DisableSafetyChecks) {
    OpaqueFunctionsPool<FunctionType *> OpaquePCInitializer(&M, false);
    OpaquePCInitializer.addFnAttribute(Attribute::ReadOnly);
    OpaquePCInitializer.addFnAttribute(Attribute::NoUnwind);

    Type *PCType = GCBI.pcReg()->getType()->getPointerElementType();
    auto *OpaqueFT = FunctionType::get(PCType, {}, false);
    OpaquePC = OpaquePCInitializer.get(OpaqueFT, OpaqueFT, "opaque_pc");
  }

  RaiseException = M.getFunction("raise_exception_helper");
  revng_assert(RaiseException != nullptr);

  // Create a bitmask to trace which CSVs need to be saved for the function
  auto *ConstantZero = ConstantInt::get(Type::getInt64Ty(Context), 0);
  RestoreCSVBitmask = new GlobalVariable(M,
                                         Type::getInt64Ty(Context),
                                         false,
                                         GlobalValue::InternalLinkage,
                                         ConstantZero,
                                         "RestoreCSVBitmask");

  // Collect functions we need to handle
  std::vector<Function *> Functions;
  for (Function &F : M)
    if (F.getName().startswith("bb."))
      Functions.push_back(&F);

  // Recreate the functions with the appropriate set of arguments
  for (Function *F : Functions) {
    const FunctionDescription &Descriptor = handleFunction(*F);
    auto *NewFunction = cast<Function>(Descriptor.Function);
    FunctionsMap[NewFunction] = Descriptor;
    OldToNew[F] = NewFunction;
  }

  unsigned I = 0;
  for (GlobalVariable *CSV : GCBI.csvs()) {
    CSVToIndex[CSV] = I;
    I++;
  }

  // Collect the list of function calls we intend to handle
  std::vector<CallInst *> RegularFunctionCalls;
  std::vector<CallInst *> HelperFunctionCalls;
  for (auto &P : FunctionsMap) {
    Function &F = *cast<Function>(P.second.Function);

    for (BasicBlock &BB : F) {
      for (Instruction &I : BB) {

        auto *FuncCall = I.getMetadata("func.call");
        if (FuncCall != nullptr) {
          CallInst *Call = cast<CallInst>(&I);
          if (isCallToHelper(Call)) {
            HelperFunctionCalls.push_back(Call);
          } else {
            RegularFunctionCalls.push_back(Call);
          }
        }

        auto *Call = dyn_cast<CallInst>(&I);

        if (Call == nullptr or isMarker(Call))
          continue;

        Value *CalledValue = Call->getCalledValue();
        Function *Callee = cast_or_null<Function>(skipCasts(CalledValue));

        if (Callee != nullptr and Callee->isIntrinsic())
          continue;

        if (FuncCall == nullptr) {
          BasicBlock *BB = Call->getParent();
          dbg << "Call " << getName(Call) << " ("
              << "in function " << getName(BB->getParent())
              << ", with unique predecessor "
              << getName(BB->getUniquePredecessor()) << ")"
              << " to " << getName(Callee) << " (now "
              << getName(OldToNew[Callee]) << ")"
              << " doesn't have func.call metadata\n";
          // revng_abort();
          continue;
        }

        if (Callee == nullptr)
          continue;
      }
    }
  }

  // Install BB to restore CSVs when an exception arises
  for (auto &[Func, Desc] : FunctionsMap)
    installUnwindBasicBlock(Func);

  // Handle function calls in isolated functions
  for (CallInst *Call : RegularFunctionCalls)
    handleRegularFunctionCall(Call);

  // Handle function calls to helpers in isolated functions
  for (CallInst *Call : HelperFunctionCalls)
    handleHelperFunctionCall(Call);

  // Handle invoke instructions in `root`
  handleRoot();

  // Promote CSVs to allocas
  replaceCSVsWithAlloca();

  // Replace call to invoke inst in helper functions
  replaceCallWithInvoke();

  // Zero out RestoreCSVBitmask before calling raise_exception_helper
  for (const auto &U : RaiseException->users()) {
    InvokeInst *II = cast<InvokeInst>(U);
    if (!II)
      continue;

    new StoreInst(Constant::getNullValue(
                    RestoreCSVBitmask->getType()->getPointerElementType()),
                  RestoreCSVBitmask,
                  II);
  }

  // Fill unwind basic blocks with CSVs
  for (auto &P : FunctionsMap) {
    Function *F = P.first;
    CSVsSerialization(F, UnwindEdgesMap[F]);
  }

  // Drop function_dispatcher
  if (FunctionDispatcher != nullptr) {
    FunctionDispatcher->deleteBody();
    ReturnInst::Create(Context,
                       BasicBlock::Create(Context, "", FunctionDispatcher));
  }

  // Drop all the old functions
  for (Function *Function : Functions)
    Function->eraseFromParent();

  // Quick and dirty DCE
  for (auto &P : FunctionsMap) {
    Function &F = *P.first;

    std::set<BasicBlock *> Reachable;
    ReversePostOrderTraversal<BasicBlock *> RPOT(&F.getEntryBlock());
    for (BasicBlock *BB : RPOT)
      Reachable.insert(BB);

    std::vector<BasicBlock *> ToDelete;
    for (BasicBlock &BB : F)
      if (Reachable.count(&BB) == 0)
        ToDelete.push_back(&BB);

    for (BasicBlock *BB : ToDelete)
      BB->dropAllReferences();

    for (BasicBlock *BB : ToDelete)
      BB->eraseFromParent();
  }

  bool Invalid;
  {
    raw_os_ostream Stream(dbg);
    Invalid = verifyModule(M, &Stream);
  }
  revng_assert(not Invalid);
}

void EnforceABIImpl::handleRoot() {
  // Handle invokes in root
  Function *Root = M.getFunction("root");
  revng_assert(Root != nullptr);

  for (BasicBlock &BB : *Root) {
    // Find invoke instruction
    auto It = BB.begin();
    auto End = BB.end();
    if (It == End)
      continue;
    auto *Invoke = dyn_cast<InvokeInst>(&*It);
    It++;

    if (It != End or Invoke == nullptr)
      continue;

    Function *Callee = OldToNew.at(Invoke->getCalledFunction());
    FunctionsSummary::FunctionDescription &Function = FunctionsMap.at(Callee);

    // Collect arguments
    IRBuilder<> Builder(Invoke);
    std::vector<Value *> Arguments;
    std::vector<GlobalVariable *> ReturnCSVs;
    for (auto &P : Function.RegisterSlots) {
      GlobalVariable *CSV = P.first;

      if (shouldEmit(P.second.Argument))
        Arguments.push_back(Builder.CreateLoad(CSV));

      if (shouldEmit(P.second.ReturnValue))
        ReturnCSVs.push_back(CSV);
    }

    BasicBlock *NormalDest;
    if (!ReturnCSVs.empty()) {
      NormalDest = BasicBlock::Create(Context,
                                      BB.getName() + "__extract_retval",
                                      Root,
                                      &BB);
      NormalDest->moveAfter(&BB);
    } else {
      NormalDest = Invoke->getNormalDest();
    }

    // Create the new invoke with the appropriate arguments
    auto *NewInvoke = Builder.CreateInvoke(Callee,
                                           NormalDest,
                                           Invoke->getUnwindDest(),
                                           Arguments);

    // Handle return values when a function is returning to
    // `root` due to an exception.
    if (!ReturnCSVs.empty()) {
      Builder.SetInsertPoint(NormalDest);

      if (ReturnCSVs.size() != 1) {
        unsigned I = 0;
        for (GlobalVariable *ReturnCSV : ReturnCSVs) {
          Builder.CreateStore(Builder.CreateExtractValue(NewInvoke, { I }),
                              ReturnCSV);
          I++;
        }
      } else {
        Builder.CreateStore(NewInvoke, ReturnCSVs[0]);
      }
      Builder.CreateBr(Invoke->getNormalDest());
    }

    // Erase the old invoke
    Invoke->eraseFromParent();
  }
}

FunctionsSummary::FunctionDescription
EnforceABIImpl::handleFunction(Function &F) {
  QuickMetadata QMD(Context);

  auto *Tuple = cast<MDTuple>(F.getMetadata("revng.func.entry"));

  FunctionDescription Description;
  SmallVector<Type *, 8> ArgumentsTypes;
  SmallVector<GlobalVariable *, 8> ArgumentCSVs;
  SmallVector<Type *, 8> ReturnTypes;
  SmallVector<GlobalVariable *, 8> ReturnCSVs;

  auto OperandsRange = QMD.extract<MDTuple *>(Tuple, 4)->operands();
  for (const MDOperand &Operand : OperandsRange) {
    auto *SlotTuple = cast<MDTuple>(Operand.get());

    auto *CSV = cast<GlobalVariable>(QMD.extract<Constant *>(SlotTuple, 0));

    StringRef ArgumentName = QMD.extract<StringRef>(SlotTuple, 1);
    auto Argument = FunctionRegisterArgument::fromName(ArgumentName);

    StringRef ReturnValueName = QMD.extract<StringRef>(SlotTuple, 2);
    auto ReturnValue = FunctionReturnValue::fromName(ReturnValueName);

    Description.RegisterSlots[CSV] = FunctionRegisterDescription{ Argument,
                                                                  ReturnValue };

    // Collect arguments
    if (shouldEmit(Argument)) {
      ArgumentsTypes.push_back(CSV->getType()->getPointerElementType());
      ArgumentCSVs.push_back(CSV);
    }

    // Collect return values
    if (shouldEmit(ReturnValue)) {
      ReturnTypes.push_back(CSV->getType()->getPointerElementType());
      ReturnCSVs.push_back(CSV);
    }
  }

  // Create the return type
  Type *ReturnType = Type::getVoidTy(Context);
  if (ReturnTypes.size() == 0)
    ReturnType = Type::getVoidTy(Context);
  else if (ReturnTypes.size() == 1)
    ReturnType = ReturnTypes[0];
  else
    ReturnType = StructType::create(ReturnTypes);

  // Create new function
  auto *NewType = FunctionType::get(ReturnType, ArgumentsTypes, false);
  auto *NewFunction = Function::Create(NewType,
                                       GlobalValue::ExternalLinkage,
                                       "",
                                       F.getParent());
  NewFunction->takeName(&F);
  NewFunction->copyAttributesFrom(&F);
  NewFunction->setMetadata("revng.func.entry",
                           F.getMetadata("revng.func.entry"));
  Description.Function = NewFunction;

  {
    unsigned I = 0;
    for (Argument &Argument : NewFunction->args())
      Argument.setName(ArgumentCSVs[I++]->getName());
  }

  // Steal body from the old function
  std::vector<BasicBlock *> Body;
  for (BasicBlock &BB : F)
    Body.push_back(&BB);
  auto &NewBody = NewFunction->getBasicBlockList();
  for (BasicBlock *BB : Body) {
    BB->removeFromParent();
    revng_assert(BB->getParent() == nullptr);
    NewBody.push_back(BB);
    revng_assert(BB->getParent() == NewFunction);
  }

  // Store arguments to CSVs
  BasicBlock &Entry = NewFunction->getEntryBlock();
  IRBuilder<> StoreBuilder(Entry.getTerminator());
  unsigned I = 0;
  for (Argument &TheArgument : NewFunction->args()) {
    auto *CSV = ArgumentCSVs[I];
    StoreBuilder.CreateStore(&TheArgument, CSV);
    I++;
  }

  // Build the return value
  if (ReturnCSVs.size() != 0) {
    for (BasicBlock &BB : *NewFunction) {
      if (auto *Return = dyn_cast<ReturnInst>(BB.getTerminator())) {
        IRBuilder<> Builder(Return);
        std::vector<Value *> ReturnValues;
        for (GlobalVariable *ReturnCSV : ReturnCSVs)
          ReturnValues.push_back(Builder.CreateLoad(ReturnCSV));

        if (ReturnTypes.size() == 1)
          Builder.CreateRet(ReturnValues[0]);
        else
          Builder.CreateAggregateRet(ReturnValues.data(), ReturnValues.size());

        Return->eraseFromParent();
      }
    }
  }

  return Description;
}

void EnforceABIImpl::handleHelperFunctionCall(CallInst *Call) {
  using namespace StackAnalysis;
  Function *Helper = cast<Function>(skipCasts(Call->getCalledValue()));

  auto UsedCSVs = GeneratedCodeBasicInfo::getCSVUsedByHelperCall(Call);
  UsedCSVs.sort();

  auto *PointeeTy = Helper->getType()->getPointerElementType();
  auto *HelperType = cast<llvm::FunctionType>(PointeeTy);

  HelperCallSite CallSite(UsedCSVs.Read, UsedCSVs.Written, CSVToIndex, Helper);
  auto It = HelperCallSites.find(CallSite);

  Function *HelperWrapper = nullptr;
  if (It != HelperCallSites.end()) {
    revng_assert(CallSite == It->first);
    HelperWrapper = It->second;
  } else {
    HelperWrapper = createHelperWrapper(Helper,
                                        UsedCSVs.Read,
                                        UsedCSVs.Written);

    // Record the new wrapper for future use
    HelperCallSites[CallSite] = HelperWrapper;
  }

  //
  // Emit call to the helper wrapper
  //
  IRBuilder<> Builder(Call);

  SmallVector<Value *, 16> NewArguments;

  // Initialize the new set of arguments with the old ones
  unsigned I = 0;
  for (Value *V : make_range(Call->arg_begin(), Call->arg_end())) {
    auto *Ty = HelperType->getParamType(I);
    NewArguments.push_back(Builder.CreateBitOrPointerCast(V, Ty));
    I++;
  }

  // Add arguments read
  for (GlobalVariable *CSV : UsedCSVs.Read)
    NewArguments.push_back(Builder.CreateLoad(CSV));

  // Emit the actual call
  Value *Result = Builder.CreateCall(HelperWrapper, NewArguments);

  bool HasOutputCSVs = UsedCSVs.Written.size() != 0;
  bool OriginalWasVoid = HelperType->getReturnType()->isVoidTy();
  if (HasOutputCSVs) {

    unsigned FirstDeserialized = 0;
    if (not OriginalWasVoid) {
      FirstDeserialized = 1;
      // RAUW the new result
      Value *HelperResult = Builder.CreateExtractValue(Result, { 0 });
      Call->replaceAllUsesWith(HelperResult);
    }

    // Restore into CSV the written registers
    for (unsigned I = 0; I < UsedCSVs.Written.size(); I++) {
      unsigned ResultIndex = { FirstDeserialized + I };
      Builder.CreateStore(Builder.CreateExtractValue(Result, ResultIndex),
                          UsedCSVs.Written[I]);
    }

  } else if (not OriginalWasVoid) {
    Call->replaceAllUsesWith(Result);
  }

  // Erase the old call
  Call->eraseFromParent();
}

void EnforceABIImpl::handleRegularFunctionCall(CallInst *Call) {
  Function *Callee = cast<Function>(skipCasts(Call->getCalledValue()));
  bool IsDirect = (Callee != FunctionDispatcher);
  if (IsDirect)
    Callee = OldToNew.at(Callee);

  using namespace StackAnalysis;
  QuickMetadata QMD(Context);

  // Temporary variable we're going to populate using metadata
  CallSiteDescription CallSite(Call, nullptr);

  auto *FuncCall = cast<MDTuple>(Call->getMetadata("func.call"));

  //
  // Collect arguments and return values
  //
  auto *Slots = cast<MDTuple>(QMD.extract<MDTuple *>(FuncCall, 0));
  for (const MDOperand &Operand : Slots->operands()) {
    auto *SlotTuple = cast<MDTuple>(Operand.get());

    auto *CSV = cast<GlobalVariable>(QMD.extract<Constant *>(SlotTuple, 0));

    StringRef ArgumentName = QMD.extract<StringRef>(SlotTuple, 1);
    auto Argument = FunctionCallRegisterArgument::fromName(ArgumentName);

    StringRef ReturnValueName = QMD.extract<StringRef>(SlotTuple, 2);
    auto ReturnValue = FunctionCallReturnValue::fromName(ReturnValueName);

    CallSite.RegisterSlots[CSV] = FunctionCallRegisterDescription{
      Argument, ReturnValue
    };
  }

  if (DisableSafetyChecks or IsDirect) {
    // The callee is a well-known callee, generate a direct call
    IRBuilder<> Builder(Call);
    generateCall(Builder, Callee, CallSite, false);

    if (DisableSafetyChecks) {
      // Create an additional store to the local %pc, so that the optimizer
      // cannot do stuff with llvm.assume.
      revng_assert(OpaquePC != nullptr);
      auto *OpaqueValue = Builder.CreateCall(OpaquePC);

      // The store here is done in the global CSV, since the
      // `replaceCSVsWithAlloca` method pass after us, and place the
      // corresponding alloca here.
      Value *PCCSV = GCBI.pcReg();
      Builder.CreateStore(OpaqueValue, PCCSV);
    }
  } else {
    // If it's an indirect call, enumerate all the compatible callees and
    // generate a call for each of them

    EnforceABILog << getName(Call) << " is an indirect call compatible with:\n";

    BasicBlock *BeforeSplit = Call->getParent();
    BasicBlock *AfterSplit = BeforeSplit->splitBasicBlock(Call);
    BeforeSplit->getTerminator()->eraseFromParent();

    IRBuilder<> Builder(BeforeSplit);
    Value *PCCSV = GCBI.pcReg();
    Value *PC = Builder.CreateLoad(PCCSV);
    BasicBlock *UnexpectedPC = findByBlockType(AfterSplit->getParent(),
                                               BlockType::UnexpectedPCBlock);
    revng_assert(UnexpectedPC != nullptr);
    SwitchInst *Switch = Builder.CreateSwitch(PC, UnexpectedPC);

    unsigned I = 0;
    unsigned Count = 0;
    for (auto &P : FunctionsMap) {
      Function *F = P.first;
      FunctionDescription &Description = P.second;

      EnforceABILog << "  " << F->getName().data() << " ";
      if (GlobalVariable *CSV = CallSite.isCompatibleWith(Description)) {
        EnforceABILog << "[No: " << CSV->getName().data() << "]";
      } else {
        EnforceABILog << "[Yes]";

        auto *Tuple = cast<MDTuple>(F->getMetadata("revng.func.entry"));
        revng_assert(Tuple != nullptr);
        auto PC = MetaAddress::fromConstant(QMD.extract<Constant *>(Tuple, 1));

        auto *Case = BasicBlock::Create(Context,
                                        "dispatchercase_" + Twine(Count),
                                        BeforeSplit->getParent(),
                                        AfterSplit);
        auto *Ty = cast<IntegerType>(PCCSV->getType()->getPointerElementType());
        Switch->addCase(ConstantInt::get(Ty, PC.asPC()), Case);

        Builder.SetInsertPoint(Case);
        generateCall(Builder, F, CallSite, true);
        Builder.CreateBr(AfterSplit);

        Count++;
      }
      EnforceABILog << DoLog;

      I++;
    }

    EnforceABILog << Count << " functions" << DoLog;
  }

  Call->eraseFromParent();
}

void EnforceABIImpl::generateCall(IRBuilder<> &Builder,
                                  Function *Callee,
                                  CallSiteDescription &CallSite,
                                  bool IsPartOfFunctionDispatcher) {
  revng_assert(Callee != nullptr);

  llvm::SmallVector<Type *, 8> ArgumentsTypes;
  llvm::SmallVector<Value *, 8> Arguments;
  llvm::SmallVector<Type *, 8> ReturnTypes;
  llvm::SmallVector<GlobalVariable *, 8> ReturnCSVs;

  Value *Result = nullptr;

  bool IsDirect = (Callee != FunctionDispatcher);
  if (not IsDirect) {
    revng_assert(DisableSafetyChecks);

    // Collect arguments, returns and their type.
    for (auto &P : CallSite.RegisterSlots) {
      GlobalVariable *CSV = P.first;

      if (shouldEmit(P.second.Argument)) {
        ArgumentsTypes.push_back(CSV->getType()->getPointerElementType());
        Arguments.push_back(Builder.CreateLoad(CSV));
      }

      if (shouldEmit(P.second.ReturnValue)) {
        ReturnTypes.push_back(CSV->getType()->getPointerElementType());
        ReturnCSVs.push_back(CSV);
      }
    }

    // Create here on the fly the indirect function that we want to call.
    // Create the return type
    Type *ReturnType = Type::getVoidTy(Context);
    if (ReturnTypes.size() == 0)
      ReturnType = Type::getVoidTy(Context);
    else if (ReturnTypes.size() == 1)
      ReturnType = ReturnTypes[0];
    else
      ReturnType = StructType::create(ReturnTypes);

    // Create a new `indirect_placeholder` function with the specific function
    // type we need
    auto *NewType = FunctionType::get(ReturnType, ArgumentsTypes, false);
    Callee = IndirectPlaceholderPool.get(NewType,
                                         NewType,
                                         "indirect_placeholder");

    Result = Builder.CreateCall(Callee, Arguments);
  } else {

    // Additional debug checks if we are not emitting an indirect call.
    revng_log(EnforceABILog,
              "Emitting call to " << getName(Callee) << " from "
                                  << getName(CallSite.Call));

    FunctionDescription &CalleeDescription = FunctionsMap.at(Callee);
    revng_assert(CalleeDescription.Function != nullptr
                 and CalleeDescription.Function->getName().startswith("bb."));
    if (GlobalVariable *CSV = CallSite.isCompatibleWith(CalleeDescription)) {
      dbg << (CallSite.Call == nullptr ? "nullptr" :
                                         getName(CallSite.Call).data())
          << " -> "
          << (Callee == nullptr ? "nullptr" : Callee->getName().data()) << ": "
          << CSV->getName().data() << "\n";
      revng_abort();
    }

    // Collect arguments, returns and their type.
    for (auto &P : CalleeDescription.RegisterSlots) {
      GlobalVariable *CSV = P.first;

      if (shouldEmit(P.second.Argument)) {
        ArgumentsTypes.push_back(CSV->getType()->getPointerElementType());
        Arguments.push_back(Builder.CreateLoad(CSV));
      }

      if (shouldEmit(P.second.ReturnValue)) {
        ReturnTypes.push_back(CSV->getType()->getPointerElementType());
        ReturnCSVs.push_back(CSV);
      }
    }

    BasicBlock *BB = Builder.GetInsertBlock();
    Function *Caller = BB->getParent();

    // If we are in a basic block of an isolated function,
    // we generate an invoke instruction. Do not emit an invoke instruction
    // if DisableSafetyChecks is on.
    if (!DisableSafetyChecks && Caller->getName().startswith("bb.")
        && !BB->getName().endswith("__continuation")) {
      BasicBlock *NormalEdge;
      std::string NormalEdgeName = std::string(BB->getName())
                                   + "__continuation";

      // If we are emitting an indirect call, we are dealing with a degenerate
      // BB (has no terminator yet), since the basic block inside the jump table
      // is still being created. Hence, we just need to create a new basic block
      if (IsPartOfFunctionDispatcher) {
        NormalEdge = BasicBlock::Create(Context,
                                        NormalEdgeName,
                                        Caller,
                                        nullptr);

        NormalEdge->moveAfter(BB);
      } else {
        NormalEdge = BB->splitBasicBlock(CallSite.Call, NormalEdgeName);
        Builder.SetInsertPoint(BB);

        // The branch created by splitBasicBlock is
        // to be replaced with an invoke
        BB->getTerminator()->eraseFromParent();
      }

      // NormalEdge cannot be null. If it is, something is wrong
      revng_assert(NormalEdge != nullptr);

      auto *UnwindEdge = UnwindEdgesMap[Caller];
      Result = Builder.CreateInvoke(CalleeDescription.Function,
                                    NormalEdge,
                                    UnwindEdge,
                                    Arguments);

      if (IsPartOfFunctionDispatcher)
        Builder.SetInsertPoint(NormalEdge);
      else
        Builder.SetInsertPoint(NormalEdge->getFirstNonPHI());

    } else {
      Result = Builder.CreateCall(CalleeDescription.Function, Arguments);
    }
  }

  if (ReturnCSVs.size() != 1) {
    unsigned I = 0;
    for (GlobalVariable *ReturnCSV : ReturnCSVs) {
      Builder.CreateStore(Builder.CreateExtractValue(Result, { I }), ReturnCSV);
      I++;
    }
  } else {
    Builder.CreateStore(Result, ReturnCSVs[0]);
  }
}

void EnforceABIImpl::replaceCSVsWithAlloca() {
  OpaqueFunctionsPool<StringRef> CSVInitializers(&M, false);
  CSVInitializers.addFnAttribute(Attribute::ReadOnly);
  CSVInitializers.addFnAttribute(Attribute::NoUnwind);

  // Each CSV has a map. The key of the map is a Funciton, and the mapped value
  // is an AllocaInst used to locally represent that CSV.
  using MapsVector = std::vector<ValueMap<Function *, Value *>>;
  MapsVector CSVMaps(GCBI.csvs().size(), {});

  // Map CSV GlobalVariable * to a position in CSVMaps
  ValueMap<llvm::GlobalVariable *, MapsVector::size_type> CSVPosition;
  for (auto &Group : llvm::enumerate(GCBI.csvs()))
    CSVPosition[Group.value()] = Group.index();

  for (auto &P : FunctionsMap) {
    Function &F = *P.first;

    // Identifies the GlobalVariables representing CSVs used in F.
    std::set<GlobalVariable *> CSVs;
    for (BasicBlock &BB : F) {
      for (Instruction &I : BB) {
        Value *Pointer = nullptr;
        if (auto *Load = dyn_cast<LoadInst>(&I))
          Pointer = Load->getPointerOperand();
        else if (auto *Store = dyn_cast<StoreInst>(&I))
          Pointer = Store->getPointerOperand();
        else
          continue;

        Pointer = skipCasts(Pointer);

        if (auto *CSV = dyn_cast_or_null<GlobalVariable>(Pointer))
          if (!(GCBI.isSPReg(CSV) || GCBI.isPCReg(CSV)))
            CSVs.insert(CSV);
      }
    }

    // Create an alloca for each CSV and replace all uses of CSVs with the
    // corresponding allocas
    BasicBlock &Entry = F.getEntryBlock();
    IRBuilder<> InitializersBuilder(&Entry, Entry.begin());
    auto *Separator = InitializersBuilder.CreateUnreachable();
    IRBuilder<> AllocaBuilder(Separator);

    // For each GlobalVariable representing a CSV used in F, create a dedicated
    // alloca and save it in CSVMaps.
    for (GlobalVariable *CSV : CSVs) {

      Type *CSVType = CSV->getType()->getPointerElementType();

      // Create and register the alloca
      auto *Alloca = AllocaBuilder.CreateAlloca(CSVType,
                                                nullptr,
                                                CSV->getName());

      // If DisableSafetyChecks is on, we initialize all allocas with opaque,
      // CSV-specific values. If off, we store a placeholder value to the alloca
      // to verify for unitialized
      if (DisableSafetyChecks) {
        auto *Initializer = CSVInitializers.get(CSV->getName(),
                                                CSVType,
                                                {},
                                                Twine("init_")
                                                  + CSV->getName());
        {
          auto &B = InitializersBuilder;
          B.CreateStore(B.CreateCall(Initializer), Alloca);
        }
      } else {
        InitializersBuilder.CreateStore(ConstantInt::get(CSVType, 0xDEADBEEF),
                                        Alloca);
      }

      // Ignore it if it's not a CSV
      auto CSVPosIt = CSVPosition.find(CSV);
      if (CSVPosIt == CSVPosition.end())
        continue;

      auto CSVPos = CSVPosIt->second;
      CSVMaps[CSVPos][&F] = Alloca;

      // Record the CSV to the global map as well.
      CSVMap[std::make_pair(CSV, &F)] = Alloca;
    }

    Separator->eraseFromParent();
  }
  // Substitute the uses of the GlobalVariables representing the CSVs with the
  // dedicate AllocaInst that were created in each Function.
  for (GlobalVariable *CSV : GCBI.csvs()) {
    auto CSVPosIt = CSVPosition.find(CSV);
    auto CSVPos = CSVPosIt->second;
    const auto &FunctionAllocas = CSVMaps.at(CSVPos);
    if (not FunctionAllocas.empty())
      replaceAllUsesInFunctionsWith(CSV, FunctionAllocas);
  }
}

void EnforceABIImpl::replaceCallWithInvoke() {
  SmallVector<Function *, 1> Helpers{ RaiseException };
  for (Function *Callee : Helpers) {
    for (const auto &U : Callee->users()) {
      // New invoke uses are pushed in front, so it is safe
      // to cast directly to CallInst
      CallInst *CI = cast<CallInst>(U);
      if (!CI)
        continue;

      BasicBlock *BB = CI->getParent();
      StringRef ContEdgeName = Callee == RaiseException ? "__unreachable" :
                                                          "__return";

      BasicBlock *NormalEdge = BB->splitBasicBlock(CI,
                                                   BB->getName()
                                                     + ContEdgeName);
      BasicBlock *UnwindEdge = UnwindEdgesMap[BB->getParent()];
      BB->getInstList().pop_back();

      // Transform `call` instructions in `invoke` jumping
      // to the unwind block in case of exception
      SmallVector<Value *, 4> InvokeArgs(CI->arg_begin(), CI->arg_end());
      InvokeInst *II = InvokeInst::Create(CI->getCalledValue(),
                                          NormalEdge,
                                          UnwindEdge,
                                          InvokeArgs,
                                          CI->getName(),
                                          BB);

      II->setDebugLoc(CI->getDebugLoc());
      II->setCallingConv(CI->getCallingConv());
      II->setAttributes(CI->getAttributes());

      // Replace it and remove the `call` inst
      CI->replaceAllUsesWith(II);
      revng_assert(CI->use_empty());

      CI->eraseFromParent();
    }
  }
}

void EnforceABIImpl::CSVsSerialization(Function *F, BasicBlock *RestoreBB) {
  IRBuilder<> Builder(RestoreBB->getTerminator());

  for (const auto &CSVObj : CSVMap) {
    const std::pair<GlobalVariable *, Function *> *CSVOfFunction = &(
      CSVObj.first);
    GlobalVariable *CSV = CSVOfFunction->first;
    Value *AI = CSVObj.second;

    if (CSVOfFunction->second != F)
      continue;

    // CSV serialization: iterate over all CSVs and check if the CSVs
    // need to be assigned their equivalent local variable. A bitmask
    // is maintained to check if the CSV has already been serialized.
    // if (!(RestoreCSVBitMask & CSV[i]) && CSV_local  != 0xDEADBEEF) {
    //   g_CSV = local_CSV;
    //   RestoreCSVBitMask |= CSV[i];
    // }
    Value *BitmaskValue = Builder.CreateLoad(RestoreCSVBitmask);

    Value *Shift = ConstantInt::get(Type::getInt64Ty(Context),
                                    1 << CSVToIndex[CSV]);

    Value *AlreadyStored = Builder.CreateAnd(BitmaskValue, Shift);
    Value *Cond1 = Builder.CreateICmpEQ(AlreadyStored, Builder.getInt64(0));

    Value *LoadAI = Builder.CreateLoad(AI);

    Value *Marker = ConstantInt::get(CSV->getType()->getPointerElementType(),
                                     0xDEADBEEF);
    Value *Cond2 = Builder.CreateICmpNE(LoadAI, Marker);

    Value *ResultCond = Builder.CreateAnd(Cond1, Cond2);

    Value *LoadCSV = Builder.CreateLoad(CSV);
    Value *Res = Builder.CreateSelect(ResultCond, LoadAI, LoadCSV);
    Builder.CreateStore(Res, CSV);

    Value *NewBitmaskValue = Builder.CreateOr(BitmaskValue, Shift);
    Res = Builder.CreateSelect(ResultCond, BitmaskValue, NewBitmaskValue);
    Builder.CreateStore(Res, RestoreCSVBitmask);
  }
}

void EnforceABIImpl::installUnwindBasicBlock(Function *F) {
  BasicBlock *UnwindEdge = BasicBlock::Create(Context,
                                              F->getName()
                                                + "__csv_restore_unwind",
                                              F);

  Type *ExnTy = StructType::get(Type::getInt8PtrTy(Context),
                                Type::getInt32Ty(Context));
  LandingPadInst *LPad = LandingPadInst::Create(ExnTy, 1, "lpad", UnwindEdge);
  LPad->setCleanup(true);

  Function *Root = M.getFunction("root");
  F->setPersonalityFn(Root->getPersonalityFn());

  // Propagate the exception to the callee.
  // No need to call `_Unwind_RaiseException` again.
  ResumeInst::Create(LPad, UnwindEdge);

  UnwindEdgesMap[F] = UnwindEdge;
}
