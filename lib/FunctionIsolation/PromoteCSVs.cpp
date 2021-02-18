/// \file PromoteCSVs.cpp
/// \brief

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/ADT/GenericGraph.h"
#include "revng/FunctionIsolation/PromoteCSVs.h"
#include "revng/FunctionIsolation/StructInitializers.h"
#include "revng/MFP/MFP.h"
#include "revng/Support/IRHelpers.h"
#include "revng/TypeShrinking/SetLattices.h"

using namespace llvm;
using namespace MFP;

char PromoteCSVsPass::ID = 0;
using Register = RegisterPass<PromoteCSVsPass>;
static Register X("promote-csvs", "Promote CSVs Pass", true, true);

// TODO: switch from CallInst to CallBase

struct CSVsUsageMap {
  using CSVsUsage = GeneratedCodeBasicInfo::CSVsUsage;

  std::map<Function *, CSVsUsage> Functions;
  std::map<CallInst *, CSVsUsage> Calls;

  CSVsUsage &get(CallInst *Call) {
    auto It = Calls.find(Call);
    if (It != Calls.end()) {
      return It->second;
    } else {
      return Functions.at(getCallee(Call));
    }
  }
};

class PromoteCSVs {
private:
  struct WrapperKey {
  public:
    Function *Helper;
    std::set<GlobalVariable *> Read;
    std::set<GlobalVariable *> Written;

  private:
    auto tie() const { return std::tie(Helper, Read, Written); }

  public:
    bool operator<(const WrapperKey &Other) const {
      return tie() < Other.tie();
    }
  };

private:
  Module *M;
  StructInitializers Initializers;
  OpaqueFunctionsPool<StringRef> CSVInitializers;
  std::map<WrapperKey, Function *> Wrappers;
  const GeneratedCodeBasicInfo &GCBI;
  std::set<GlobalVariable *> CSVs;

public:
  PromoteCSVs(Module *M, const GeneratedCodeBasicInfo &GCBI);

public:
  void run();

private:
  void wrap(CallInst *Call,
            ArrayRef<GlobalVariable *> Read,
            ArrayRef<GlobalVariable *> Written);
  void promoteCSVs(Function *F);
  Function *createWrapper(const WrapperKey &Key);
  CSVsUsageMap getUsedCSVs(ArrayRef<CallInst *> CallsRange);
  void wrapCallsToHelpers(Function *F);
};

PromoteCSVs::PromoteCSVs(Module *M, const GeneratedCodeBasicInfo &GCBI) :
  M(M), Initializers(M), CSVInitializers(M, false), GCBI(GCBI) {

  CSVInitializers.addFnAttribute(Attribute::ReadOnly);
  CSVInitializers.addFnAttribute(Attribute::NoUnwind);
  CSVInitializers.setTags({ &FunctionTags::OpaqueCSVValue });

  // Record existing initializers
  for (GlobalVariable *CSV : GCBI.csvs())
    if (auto *F = M->getFunction((Twine("init_") + CSV->getName()).str()))
      if (FunctionTags::OpaqueCSVValue.isTagOf(F))
        CSVInitializers.record(CSV->getName(), F);

  copy(GCBI.csvs(), std::inserter(this->CSVs, this->CSVs.begin()));
}

// TODO: assign alias information
Function *PromoteCSVs::createWrapper(const WrapperKey &Key) {
  auto &[Helper, Read, Written] = Key;

  LLVMContext &Context = Helper->getParent()->getContext();
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
  HelperWrapper->setSection(Helper->getSection());

  // Copy and extend tags
  auto Tags = FunctionTags::TagsSet::from(Helper);
  Tags.insert(&FunctionTags::CSVsAsArgumentsWrapper);
  Tags.addTo(HelperWrapper);

  auto *Entry = BasicBlock::Create(Context, "", HelperWrapper);

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

    Initializers.createReturn(Builder, ReturnValues);

  } else if (OriginalWasVoid) {
    Builder.CreateRetVoid();
  } else {
    Builder.CreateRet(HelperResult);
  }

  return HelperWrapper;
}

template<typename T>
std::set<T> toSet(ArrayRef<T> AR) {
  std::set<T> Result;
  copy(AR, std::inserter(Result, Result.begin()));
  return Result;
}

void PromoteCSVs::wrap(CallInst *Call,
                       ArrayRef<GlobalVariable *> Read,
                       ArrayRef<GlobalVariable *> Written) {

  if (Read.size() == 0 and Written.size() == 0)
    return;

  Function *Helper = getCallee(Call);
  revng_assert(Helper != nullptr);

  WrapperKey Key{ Helper, toSet(Read), toSet(Written) };

  // Fetch or create the wrapper
  Function *&HelperWrapper = Wrappers[Key];
  if (HelperWrapper == nullptr)
    HelperWrapper = createWrapper(Key);

  auto *PointeeTy = Helper->getType()->getPointerElementType();
  auto *HelperType = cast<FunctionType>(PointeeTy);

  //
  // Emit call to the helper wrapper
  //
  IRBuilder<> Builder(Call);

  // Initialize the new set of arguments with the old ones
  SmallVector<Value *, 16> NewArguments;
  for (auto [Argument, Type] : zip(Call->args(), HelperType->params()))
    NewArguments.push_back(Builder.CreateBitOrPointerCast(Argument, Type));

  // Add arguments read
  for (GlobalVariable *CSV : Read)
    NewArguments.push_back(Builder.CreateLoad(CSV));

  // Emit the actual call
  Instruction *Result = Builder.CreateCall(HelperWrapper, NewArguments);
  Result->setDebugLoc(Call->getDebugLoc());

  bool HasOutputCSVs = Written.size() != 0;
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
    for (unsigned I = 0; I < Written.size(); I++) {
      unsigned ResultIndex = { FirstDeserialized + I };
      Builder.CreateStore(Builder.CreateExtractValue(Result, ResultIndex),
                          Written[I]);
    }

  } else if (not OriginalWasVoid) {
    Call->replaceAllUsesWith(Result);
  }

  // Erase the old call
  Call->eraseFromParent();
}

static Instruction *findFirstNonAlloca(BasicBlock *BB) {
  for (Instruction &I : *BB)
    if (not isa<AllocaInst>(&I))
      return &I;
  return nullptr;
}

void PromoteCSVs::promoteCSVs(Function *F) {
  // Create an alloca for each CSV and replace all uses of CSVs with the
  // corresponding allocas
  BasicBlock &Entry = F->getEntryBlock();

  // Get/create initializers
  std::map<Function *, GlobalVariable *> CSVForInitializer;
  std::map<GlobalVariable *, Function *> InitializerForCSV;
  for (GlobalVariable *CSV : CSVs) {
    // Initialize all allocas with opaque, CSV-specific values
    Type *CSVType = CSV->getType()->getPointerElementType();
    auto *Initializer = CSVInitializers.get(CSV->getName(),
                                            CSVType,
                                            {},
                                            Twine("init_") + CSV->getName());
    CSVForInitializer[Initializer] = CSV;
    InitializerForCSV[CSV] = Initializer;
  }

  // Collect existing initializer calls
  std::map<GlobalVariable *, CallInst *> InitializerCalls;
  for (Instruction &I : Entry) {
    if (auto *Call = dyn_cast<CallInst>(&I)) {
      auto It = CSVForInitializer.find(Call->getCalledFunction());
      if (It != CSVForInitializer.end()) {
        InitializerCalls[It->second] = Call;
      }
    }
  }

  Instruction *NonAlloca = findFirstNonAlloca(&Entry);
  revng_assert(NonAlloca != nullptr);

  IRBuilder<> AllocaBuilder(&Entry, NonAlloca->getIterator());
  auto *Separator = AllocaBuilder.CreateUnreachable();
  IRBuilder<> InitializersBuilder(&Entry, ++Separator->getIterator());

  // For each GlobalVariable representing a CSV used in F, create a dedicated
  // alloca and save it in CSVMaps.
  for (GlobalVariable *CSV : CSVs) {

    Type *CSVType = CSV->getType()->getPointerElementType();

    // Create the alloca
    auto *Alloca = AllocaBuilder.CreateAlloca(CSVType, nullptr, CSV->getName());

    // Check if already have an initializer
    CallInst *InitializerCall = nullptr;
    auto It = InitializerCalls.find(CSV);
    if (It == InitializerCalls.end()) {
      Function *Initializer = InitializerForCSV.at(CSV);
      InitializerCall = InitializersBuilder.CreateCall(Initializer);
    } else {
      InitializerCall = It->second;
    }

    // Initialize the alloca
    InitializersBuilder.SetInsertPoint(&Entry,
                                       ++InitializerCall->getIterator());
    InitializersBuilder.CreateStore(InitializerCall, Alloca);

    // Replace users
    replaceAllUsesInFunctionWith(F, CSV, Alloca);
  }

  // Drop separators
  Separator->eraseFromParent();
}

struct FunctionNodeData {
  Function *F;
  using UsedCSVSet = std::set<std::pair<bool, GlobalVariable *>>;
  UsedCSVSet UsedCSVs;
};

using FunctionNode = ForwardNode<FunctionNodeData>;
using GenericCallGraph = GenericGraph<FunctionNode>;

static FunctionNode *getNode(std::map<Function *, FunctionNode *> &NodeMap,
                             GenericCallGraph &Graph,
                             Function *F) {
  FunctionNode *Result = nullptr;

  auto It = NodeMap.find(F);
  if (It == NodeMap.end()) {
    Result = Graph.addNode();
    Result->F = F;
    NodeMap[F] = Result;
  } else {
    Result = It->second;
  }

  return Result;
}

static void addEdge(FunctionNode *Source, FunctionNode *Destination) {

  for (auto *Successor : Source->successors())
    if (Successor == Destination)
      return;

  Source->addSuccessor(Destination);
}

static bool needsWrapper(Function *F) {
  // Ignore lifted functions and functions that have already been wrapped
  {
    using namespace FunctionTags;
    auto Tags = TagsSet::from(F);
    if (Tags.contains(Lifted) or Tags.contains(CSVsAsArgumentsWrapper)
        or Tags.contains(Marker) or Tags.contains(Exceptional))
      return false;
  }

  if (F->isIntrinsic())
    return false;

  auto IsPointer = [](Type *T) { return T->isPointerTy(); };

  return any_of(F->getFunctionType()->params(), IsPointer);
}

struct UsedRegistersMFI : public SetUnionLattice<FunctionNodeData::UsedCSVSet> {
  using Label = FunctionNode *;
  using GraphType = GenericCallGraph *;

  static LatticeElement
  applyTransferFunction(Label L, const LatticeElement &Value) {
    return combineValues(L->UsedCSVs, Value);
  }
};

CSVsUsageMap PromoteCSVs::getUsedCSVs(ArrayRef<CallInst *> CallsRange) {
  CSVsUsageMap Result;

  // Note: this graph goes from callee to callers
  GenericCallGraph CallGraph;

  std::map<Function *, FunctionNode *> NodeMap;

  // Inspect the calls we need to analyze
  //
  // There are two type of calls: calls to helpers tagged by CSAA and calls to
  // regular functions. For the former, we ask GCBI to extract the information
  // from metadata. For the latter, we use a monotone framework to compute the
  // set of read/written registers by the callee.  Note that the former is more
  // accurate thanks to CSAA being call-site sensitive.
  std::queue<Function *> Queue;
  for (CallInst *Call : CallsRange) {
    Function *Callee = getCallee(Call);
    if (FunctionTags::Helper.isTagOf(Callee)) {
      CSVsUsageMap::CSVsUsage &Usage = Result.Calls[Call];
      auto UsedCSVs = GCBI.getCSVUsedByHelperCall(Call);
      Usage.Read = UsedCSVs.Read;
      Usage.Written = UsedCSVs.Written;
    } else {
      Queue.push(Callee);
    }
  }

  while (not Queue.empty()) {
    Function *F = Queue.front();
    Queue.pop();

    auto *CallerNode = getNode(NodeMap, CallGraph, F);

    for (BasicBlock &BB : *F) {

      for (Instruction &I : BB) {
        bool Write = false;
        GlobalVariable *CSV = nullptr;

        if (auto *Store = dyn_cast<StoreInst>(&I)) {

          // Record store
          Write = true;
          CSV = dyn_cast<GlobalVariable>(skipCasts(Store->getPointerOperand()));

        } else if (auto *Load = dyn_cast<StoreInst>(&I)) {

          // Record load
          CSV = dyn_cast<GlobalVariable>(skipCasts(Store->getPointerOperand()));

        } else if (auto *Call = dyn_cast<CallInst>(&I)) {
          Function *Callee = getCallee(Call);
          revng_assert(Callee != nullptr);

          // In case we meet an `abort` skip this block
          if (Callee->getName() == "abort")
            break;

          // TODO: use forwardTaintAnalysis
          if (not needsWrapper(Callee))
            continue;

          // Ensure callee is visited
          if (NodeMap.count(Callee) == 0)
            Queue.push(Callee);

          // Insert an edge in the call graph
          auto *CalleeNode = getNode(NodeMap, CallGraph, Callee);
          addEdge(CalleeNode, CallerNode);
        }

        // If there was a memory access targeting a CSV, record it
        if (CSVs.count(CSV) != 0) {
          CallerNode->UsedCSVs.insert({ Write, CSV });
        }
      }
    }
  }

  auto AnalysisResult = getMaximalFixedPoint<UsedRegistersMFI>({},
                                                               &CallGraph,
                                                               {},
                                                               {},
                                                               {},
                                                               {});

  // Populate results set
  for (auto &[Label, Value] : AnalysisResult) {
    auto &FunctionDescriptor = Result.Functions[Label->F];
    for (auto [IsWrite, CSV] : Value.OutValue) {
      if (IsWrite)
        FunctionDescriptor.Written.push_back(CSV);
      else
        FunctionDescriptor.Read.push_back(CSV);
    }
  }

  return Result;
}

template<typename T>
ArrayRef<T> oneElement(T &Element) {
  return ArrayRef(&Element, 1);
}

void PromoteCSVs::wrapCallsToHelpers(Function *F) {
  std::vector<CallInst *> ToWrap;
  for (BasicBlock &BB : *F) {
    for (Instruction &I : BB) {
      if (auto *Call = dyn_cast<CallInst>(&I)) {
        Function *Callee = getCallee(Call);
        revng_assert(Callee != nullptr);

        // Ignore calls to isolated functions
        if (not needsWrapper(Callee))
          continue;

        ToWrap.emplace_back(Call);
      }
    }
  }

  auto UsedCSVs = getUsedCSVs(ToWrap);

  for (CallInst *Call : ToWrap) {
    CSVsUsageMap::CSVsUsage &CSVsUsage = UsedCSVs.get(Call);

    // Sort to ensure compatibility between caller and callee
    CSVsUsage.sort();

    wrap(Call, CSVsUsage.Read, CSVsUsage.Written);
  }
}

void PromoteCSVs::run() {
  for (Function &F : FunctionTags::Lifted.functions(M)) {
    // Wrap calls to wrappers
    wrapCallsToHelpers(&F);

    // (Re-)promote CSVs
    promoteCSVs(&F);
  }
}

bool PromoteCSVsPass::runOnModule(Module &M) {
  auto &GCBI = getAnalysis<GeneratedCodeBasicInfoWrapperPass>().getGCBI();
  PromoteCSVs HW(&M, GCBI);
  HW.run();
  return true;
}
