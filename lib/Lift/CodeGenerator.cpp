/// \file CodeGenerator.cpp
/// \brief This file handles the whole translation process from the input
///        assembly to LLVM IR.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstring>
#include <fstream>
#include <memory>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/ExecutionEngine/RuntimeDyld.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/DiagnosticPrinter.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Linker/Linker.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/raw_os_ostream.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Utils.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "revng/FunctionCallIdentification/FunctionCallIdentification.h"
#include "revng/FunctionCallIdentification/PruneRetSuccessors.h"
#include "revng/Lift/CodeGenerator.h"
#include "revng/Lift/ExternalJumpsHandler.h"
#include "revng/Lift/InstructionTranslator.h"
#include "revng/Lift/JumpTargetManager.h"
#include "revng/Lift/PTCInterface.h"
#include "revng/Lift/VariableManager.h"
#include "revng/Model/SerializeModelPass.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/Debug.h"
#include "revng/Support/DebugHelper.h"
#include "revng/Support/FunctionTags.h"
#include "revng/Support/ProgramCounterHandler.h"
#include "revng/Support/revng.h"

#include "CodeGenerator.h"
#include "ExternalJumpsHandler.h"
#include "InstructionTranslator.h"
#include "JumpTargetManager.h"
#include "PTCInterface.h"
#include "VariableManager.h"

using namespace llvm;

using std::make_pair;
using std::string;

// Register all the arguments

// TODO: can we drop this and the associated functionality?
static cl::opt<string> CoveragePath("coverage-path",
                                    cl::desc("destination path for the CSV "
                                             "containing "
                                             "translated ranges"),
                                    cl::value_desc("path"),
                                    cl::cat(MainCategory));
static cl::alias A1("c",
                    cl::desc("Alias for -coverage-path"),
                    cl::aliasopt(CoveragePath),
                    cl::cat(MainCategory));

// TODO: linking-info-path?
static cl::opt<string> LinkingInfoPath("linking-info",
                                       cl::desc("destination path for the CSV "
                                                "containing linking info"),
                                       cl::value_desc("path"),
                                       cl::cat(MainCategory));
static cl::alias A2("i",
                    cl::desc("Alias for -linking-info"),
                    cl::aliasopt(LinkingInfoPath),
                    cl::cat(MainCategory));

// TODO: can we drop this and the associated functionality?
static cl::opt<string> BBSummaryPath("bb-summary",
                                     cl::desc("destination path for the CSV "
                                              "containing the statistics about "
                                              "the translated basic blocks"),
                                     cl::value_desc("path"),
                                     cl::cat(MainCategory));
static cl::alias A3("b",
                    cl::desc("Alias for -bb-summary"),
                    cl::aliasopt(BBSummaryPath),
                    cl::cat(MainCategory));

// Enable Debug Options to be specified on the command line
namespace DIT = DebugInfoType;
static auto X = cl::values(clEnumValN(DIT::None,
                                      "none",
                                      "no debug information"),
                           clEnumValN(DIT::OriginalAssembly,
                                      "asm",
                                      "debug information referred to the "
                                      "assembly "
                                      "of the input file"),
                           clEnumValN(DIT::PTC,
                                      "ptc",
                                      "debug information referred to the "
                                      "Portable "
                                      "Tiny Code"),
                           clEnumValN(DIT::LLVMIR,
                                      "ll",
                                      "debug information referred to the LLVM "
                                      "IR"));
static cl::opt<DIT::Values> DebugInfo("debug-info",
                                      cl::desc("emit debug information"),
                                      X,
                                      cl::cat(MainCategory),
                                      cl::init(DIT::LLVMIR));

static cl::alias A6("g",
                    cl::desc("Alias for -debug-info"),
                    cl::aliasopt(DebugInfo),
                    cl::cat(MainCategory));

// TODO: is this still active?
static cl::opt<string> DebugPath("debug-path",
                                 cl::desc("destination path for the generated "
                                          "debug source"),
                                 cl::value_desc("path"),
                                 cl::cat(MainCategory));

static cl::opt<bool> RecordPTC("record-ptc",
                               cl::desc("create metadata for PTC"),
                               cl::cat(MainCategory));

static Logger<> PTCLog("ptc");

template<typename T, typename... Args>
inline std::array<T, sizeof...(Args)> make_array(Args &&...args) {
  return { { std::forward<Args>(args)... } };
}

/// Wrap a value around a temporary opaque function
///
/// Useful to prevent undesired optimizations
class OpaqueIdentity {
private:
  std::map<Type *, Function *> Map;
  Module *M;

public:
  OpaqueIdentity(Module *M) : M(M) {}

  ~OpaqueIdentity() { revng_assert(Map.size() == 0); }

  void drop() {
    SmallVector<CallInst *, 16> ToErase;
    for (auto [T, F] : Map) {
      for (User *U : F->users()) {
        auto *Call = cast<CallInst>(U);
        Call->replaceAllUsesWith(Call->getArgOperand(0));
        ToErase.push_back(Call);
      }
    }

    for (CallInst *Call : ToErase)
      Call->eraseFromParent();

    for (auto [T, F] : Map)
      F->eraseFromParent();

    Map.clear();
  }

  Instruction *wrap(IRBuilder<> &Builder, Value *V) {
    Type *ResultType = V->getType();
    Function *F = nullptr;
    auto It = Map.find(ResultType);
    if (It == Map.end()) {
      auto *FT = FunctionType::get(ResultType, { ResultType }, false);
      F = Function::Create(FT, GlobalValue::ExternalLinkage, "id", *M);
      F->addFnAttr(Attribute::ReadOnly);
      Map[ResultType] = F;
    } else {
      F = It->second;
    }

    return Builder.CreateCall(F, { V });
  }

  Instruction *wrap(Instruction *I) {
    IRBuilder<> Builder(I->getParent(), ++I->getIterator());
    return wrap(Builder, I);
  }
};

// Outline the destructor for the sake of privacy in the header
CodeGenerator::~CodeGenerator() = default;

static std::unique_ptr<Module> parseIR(StringRef Path, LLVMContext &Context) {
  std::unique_ptr<Module> Result;
  SMDiagnostic Errors;
  Result = parseIRFile(Path, Errors, Context);

  if (Result.get() == nullptr) {
    Errors.print("revng", dbgs());
    revng_abort();
  }

  return Result;
}

CodeGenerator::CodeGenerator(BinaryFile &Binary,
                             Architecture &Target,
                             llvm::LLVMContext &TheContext,
                             std::string Output,
                             std::string Helpers,
                             std::string EarlyLinked) :
  TargetArchitecture(std::move(Target)),
  Context(TheContext),
  TheModule(new Module("top", Context)),
  OutputPath(Output),
  Debug(new DebugHelper(Output, TheModule.get(), DebugInfo, DebugPath)),
  Binary(Binary) {

  OriginalInstrMDKind = Context.getMDKindID("oi");
  PTCInstrMDKind = Context.getMDKindID("pi");

  HelpersModule = parseIR(Helpers, Context);

  TheModule->setDataLayout(HelpersModule->getDataLayout());

  // Tag all global objects in HelpersModule as QEMU
  for (GlobalVariable &G : HelpersModule->globals())
    FunctionTags::QEMU.addTo(&G);

  for (Function &F : HelpersModule->functions()) {
    F.setDSOLocal(false);

    FunctionTags::QEMU.addTo(&F);

    if (F.hasFnAttribute(Attribute::NoReturn)
        or F.getSection() == "revng_exceptional")
      FunctionTags::Exceptional.addTo(&F);
  }

  EarlyLinkedModule = parseIR(EarlyLinked, Context);

  if (CoveragePath.size() == 0)
    CoveragePath = Output + ".coverage.csv";

  if (BBSummaryPath.size() == 0)
    BBSummaryPath = Output + ".bbsummary.csv";

  // Prepare the linking info CSV
  if (LinkingInfoPath.size() == 0)
    LinkingInfoPath = OutputPath + ".li.csv";
  std::ofstream LinkingInfoStream(LinkingInfoPath);
  LinkingInfoStream << "name,start,end\n";

  auto *Uint8Ty = Type::getInt8Ty(Context);
  auto *ElfHeaderHelper = new GlobalVariable(*TheModule,
                                             Uint8Ty,
                                             true,
                                             GlobalValue::ExternalLinkage,
                                             ConstantInt::get(Uint8Ty, 0),
                                             "elfheaderhelper");
  ElfHeaderHelper->setAlignment(MaybeAlign(1));
  ElfHeaderHelper->setSection(".elfheaderhelper");

  auto *RegisterType = Type::getIntNTy(Context,
                                       Binary.architecture().pointerSize());
  auto createConstGlobal = [this, &RegisterType](const Twine &Name,
                                                 uint64_t Value) {
    return new GlobalVariable(*TheModule,
                              RegisterType,
                              true,
                              GlobalValue::ExternalLinkage,
                              ConstantInt::get(RegisterType, Value),
                              Name);
  };

  // These values will be used to populate the auxiliary vectors
  if (Binary.programHeadersAddress().isValid()) {
    createConstGlobal("e_phentsize", Binary.programHeaderSize());
    createConstGlobal("e_phnum", Binary.programHeadersCount());
    createConstGlobal("phdr_address", Binary.programHeadersAddress().address());
  }

  for (SegmentInfo &Segment : Binary.segments()) {
    // If it's executable register it as a valid code area
    if (Segment.IsExecutable) {
      // We ignore possible p_filesz-p_memsz mismatches, zeros wouldn't be
      // useful code anyway
      size_t Size = static_cast<size_t>(Segment.Data.size());
      bool Success = ptc.mmap(Segment.StartVirtualAddress.address(),
                              static_cast<const void *>(Segment.Data.data()),
                              Size);
      if (not Success) {
        dbg << "Couldn't mmap segment starting at ";
        Segment.StartVirtualAddress.dump(dbg);
        dbg << " with size 0x" << Size << "\n";
        revng_abort();
      }

      bool Found = false;
      MetaAddress End = Segment.pagesRange().second;
      for (SegmentInfo &Segment : Binary.segments()) {
        if (Segment.IsExecutable and Segment.containsInPages(End)) {
          Found = true;
          break;
        }
      }

      // The next page is not mapped
      if (not Found) {
        revng_check(Segment.EndVirtualAddress.address() != 0);
        NoMoreCodeBoundaries.insert(Segment.EndVirtualAddress);
        const auto &Architecture = Binary.architecture();
        auto BasicBlockEndingPattern = Architecture.basicBlockEndingPattern();
        ptc.mmap(End.address(),
                 BasicBlockEndingPattern.data(),
                 BasicBlockEndingPattern.size());
      }
    }

    std::string Name = Segment.generateName();

    // Get data and size
    auto *DataType = ArrayType::get(Uint8Ty, Segment.size());

    Constant *TheData = nullptr;
    if (Segment.size() == Segment.Data.size()) {
      // Create the array directly from the mmap'd ELF
      TheData = ConstantDataArray::get(Context, Segment.Data);
    } else {
      // If we have extra data at the end we need to create a copy of the
      // segment and append the NULL bytes
      auto FullData = std::make_unique<uint8_t[]>(Segment.size());

      size_t MinSize = std::min(Segment.size(), Segment.Data.size());
      ::memcpy(FullData.get(), Segment.Data.data(), MinSize);
      if (Segment.size() > Segment.Data.size())
        ::bzero(FullData.get() + Segment.Data.size(),
                Segment.size() - Segment.Data.size());
      auto DataRef = ArrayRef<uint8_t>(FullData.get(), Segment.size());
      TheData = ConstantDataArray::get(Context, DataRef);
    }

    // Create a new global variable
    Segment.Variable = new GlobalVariable(*TheModule,
                                          DataType,
                                          !Segment.IsWriteable,
                                          GlobalValue::ExternalLinkage,
                                          TheData,
                                          Name);

    // Force alignment to 1 and assign the variable to a specific section
    Segment.Variable->setAlignment(MaybeAlign(1));
    Segment.Variable->setSection("." + Name);

    // Write the linking info CSV
    LinkingInfoStream << "." << Name << ",0x" << std::hex
                      << Segment.StartVirtualAddress.address() << ",0x"
                      << std::hex << Segment.EndVirtualAddress.address()
                      << "\n";
  }

  // Write needed libraries CSV
  std::string NeededLibs = OutputPath + ".need.csv";
  std::ofstream NeededLibsStream(NeededLibs);
  for (const std::string &Library : Binary.neededLibraryNames())
    NeededLibsStream << Library << "\n";
}

std::string SegmentInfo::generateName() {
  // Create name from start and size
  std::stringstream NameStream;
  NameStream << "o_" << (IsReadable ? "r" : "") << (IsWriteable ? "w" : "")
             << (IsExecutable ? "x" : "") << "_0x" << std::hex
             << StartVirtualAddress.address();

  return NameStream.str();
}

static BasicBlock *replaceFunction(Function *ToReplace) {
  // Save metadata
  SmallVector<std::pair<unsigned, MDNode *>, 4> SavedMetadata;
  ToReplace->getAllMetadata(SavedMetadata);

  ToReplace->setLinkage(GlobalValue::InternalLinkage);
  ToReplace->dropAllReferences();

  // Restore metadata
  for (auto [Kind, MD] : SavedMetadata)
    ToReplace->setMetadata(Kind, MD);

  return BasicBlock::Create(ToReplace->getParent()->getContext(),
                            "",
                            ToReplace);
}

static void replaceFunctionWithRet(Function *ToReplace, uint64_t Result) {
  if (ToReplace == nullptr)
    return;

  BasicBlock *Body = replaceFunction(ToReplace);
  Value *ResultValue;

  if (ToReplace->getReturnType()->isVoidTy()) {
    revng_assert(Result == 0);
    ResultValue = nullptr;
  } else if (ToReplace->getReturnType()->isIntegerTy()) {
    auto *ReturnType = cast<IntegerType>(ToReplace->getReturnType());
    ResultValue = ConstantInt::get(ReturnType, Result, false);
  } else {
    revng_unreachable("No-op functions can only return void or an integer "
                      "type");
  }

  ReturnInst::Create(ToReplace->getParent()->getContext(), ResultValue, Body);
}

class CpuLoopFunctionPass : public llvm::ModulePass {
private:
  intptr_t ExceptionIndexOffset;

public:
  static char ID;

  CpuLoopFunctionPass() : llvm::ModulePass(ID), ExceptionIndexOffset(0) {}

  CpuLoopFunctionPass(intptr_t ExceptionIndexOffset) :
    llvm::ModulePass(ID), ExceptionIndexOffset(ExceptionIndexOffset) {}

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;

  bool runOnModule(llvm::Module &M) override;
};

char CpuLoopFunctionPass::ID = 0;

using RegisterCLF = RegisterPass<CpuLoopFunctionPass>;
static RegisterCLF Y("cpu-loop", "cpu_loop FunctionPass", false, false);

void CpuLoopFunctionPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
  AU.addRequired<LoopInfoWrapperPass>();
}

template<class Range, class UnaryPredicate>
auto find_unique(Range &&TheRange, UnaryPredicate Predicate)
  -> decltype(*TheRange.begin()) {

  const auto Begin = TheRange.begin();
  const auto End = TheRange.end();

  auto It = std::find_if(Begin, End, Predicate);
  auto Result = It;
  revng_assert(Result != End);
  revng_assert(std::find_if(++It, End, Predicate) == End);

  return *Result;
}

template<class Range>
auto find_unique(Range &&TheRange) -> decltype(*TheRange.begin()) {

  const auto Begin = TheRange.begin();
  const auto End = TheRange.end();

  auto Result = Begin;
  revng_assert(Begin != End && ++Result == End);

  return *Begin;
}

bool CpuLoopFunctionPass::runOnModule(Module &M) {
  Function &F = *M.getFunction("cpu_loop");

  // cpu_loop must return void
  revng_assert(F.getReturnType()->isVoidTy());

  Module *TheModule = F.getParent();

  // Part 1: remove the backedge of the main infinite loop
  const LoopInfo &LI = getAnalysis<LoopInfoWrapperPass>(F).getLoopInfo();
  const Loop *OutermostLoop = find_unique(LI);

  BasicBlock *Header = OutermostLoop->getHeader();

  // Check that the header has only one predecessor inside the loop
  auto IsInLoop = [&OutermostLoop](BasicBlock *Predecessor) {
    return OutermostLoop->contains(Predecessor);
  };
  BasicBlock *Footer = find_unique(predecessors(Header), IsInLoop);

  // Assert on the type of the last instruction (branch or brcond)
  revng_assert(Footer->end() != Footer->begin());
  Instruction *LastInstruction = &*--Footer->end();
  revng_assert(isa<BranchInst>(LastInstruction));

  // Remove the last instruction and replace it with a ret
  LastInstruction->eraseFromParent();
  ReturnInst::Create(F.getParent()->getContext(), Footer);

  // Part 2: replace the call to cpu_*_exec with exception_index
  auto IsCpuExec = [](Function &TheFunction) {
    StringRef Name = TheFunction.getName();
    return Name.startswith("cpu_") && Name.endswith("_exec");
  };
  Function &CpuExec = find_unique(F.getParent()->functions(), IsCpuExec);

  User *CallUser = find_unique(CpuExec.users(), [&F](User *TheUser) {
    auto *TheInstruction = dyn_cast<Instruction>(TheUser);

    if (TheInstruction == nullptr)
      return false;

    return TheInstruction->getParent()->getParent() == &F;
  });

  auto *Call = cast<CallInst>(CallUser);
  revng_assert(Call->getCalledFunction() == &CpuExec);
  Value *CPUState = Call->getArgOperand(0);
  Type *TargetType = CpuExec.getReturnType()->getPointerTo();

  IRBuilder<> Builder(Call);
  Type *IntPtrTy = Builder.getIntPtrTy(TheModule->getDataLayout());
  Value *CPUIntPtr = Builder.CreatePtrToInt(CPUState, IntPtrTy);
  using CI = ConstantInt;
  auto Offset = CI::get(IntPtrTy, ExceptionIndexOffset);
  Value *ExceptionIndexIntPtr = Builder.CreateAdd(CPUIntPtr, Offset);
  Value *ExceptionIndexPtr = Builder.CreateIntToPtr(ExceptionIndexIntPtr,
                                                    TargetType);
  Value *ExceptionIndex = Builder.CreateLoad(ExceptionIndexPtr);
  Call->replaceAllUsesWith(ExceptionIndex);
  Call->eraseFromParent();

  return true;
}

class CpuLoopExitPass : public llvm::ModulePass {
public:
  static char ID;

  CpuLoopExitPass() : llvm::ModulePass(ID), VM(nullptr) {}
  CpuLoopExitPass(VariableManager *VM) : llvm::ModulePass(ID), VM(VM) {}

  bool runOnModule(llvm::Module &M) override;

private:
  VariableManager *VM;
};

char CpuLoopExitPass::ID = 0;

using RegisterCLE = RegisterPass<CpuLoopExitPass>;
static RegisterCLE Z("cpu-loop-exit", "cpu_loop_exit Pass", false, false);

static void purgeNoReturn(Function *F) {
  auto &Context = F->getParent()->getContext();

  if (F->hasFnAttribute(Attribute::NoReturn))
    F->removeFnAttr(Attribute::NoReturn);

  for (User *U : F->users())
    if (auto *Call = dyn_cast<CallInst>(U))
      if (Call->hasFnAttr(Attribute::NoReturn)) {
        auto OldAttr = Call->getAttributes();
        auto NewAttr = OldAttr.removeAttribute(Context,
                                               AttributeList::FunctionIndex,
                                               Attribute::NoReturn);
        Call->setAttributes(NewAttr);
      }
}

static ReturnInst *createRet(Instruction *Position) {
  Function *F = Position->getParent()->getParent();
  purgeNoReturn(F);

  Type *ReturnType = F->getFunctionType()->getReturnType();
  if (ReturnType->isVoidTy()) {
    return ReturnInst::Create(F->getParent()->getContext(), nullptr, Position);
  } else if (ReturnType->isIntegerTy()) {
    auto *Zero = ConstantInt::get(static_cast<IntegerType *>(ReturnType), 0);
    return ReturnInst::Create(F->getParent()->getContext(), Zero, Position);
  } else {
    revng_assert("Return type not supported");
  }

  return nullptr;
}

/// Find all calls to cpu_loop_exit and replace them with:
///
/// * call cpu_loop
/// * set cpu_loop_exiting = true
/// * return
///
/// Then look for all the callers of the function calling cpu_loop_exit and make
/// them check whether they should return immediately (cpu_loop_exiting == true)
/// or not.
/// Then when we reach the root function, set cpu_loop_exiting to false after
/// the call.
bool CpuLoopExitPass::runOnModule(llvm::Module &M) {
  LLVMContext &Context = M.getContext();
  Function *CpuLoopExit = M.getFunction("cpu_loop_exit");

  // Nothing to do here
  if (CpuLoopExit == nullptr)
    return false;

  revng_assert(VM->hasEnv());

  purgeNoReturn(CpuLoopExit);

  Function *CpuLoop = M.getFunction("cpu_loop");
  IntegerType *BoolType = Type::getInt1Ty(Context);
  std::set<Function *> FixedCallers;
  Constant *CpuLoopExitingVariable = nullptr;
  CpuLoopExitingVariable = new GlobalVariable(M,
                                              BoolType,
                                              false,
                                              GlobalValue::CommonLinkage,
                                              ConstantInt::getFalse(BoolType),
                                              StringRef("cpu_loop_exiting"));

  revng_assert(CpuLoop != nullptr);

  std::queue<User *> CpuLoopExitUsers;
  for (User *TheUser : CpuLoopExit->users())
    CpuLoopExitUsers.push(TheUser);

  while (!CpuLoopExitUsers.empty()) {
    auto *Call = cast<CallInst>(CpuLoopExitUsers.front());
    CpuLoopExitUsers.pop();
    revng_assert(Call->getCalledFunction() == CpuLoopExit);

    // Call cpu_loop
    auto *FirstArgTy = CpuLoop->getFunctionType()->getParamType(0);
    auto *EnvPtr = VM->CPUStateToEnv(Call->getArgOperand(0), FirstArgTy, Call);

    auto *CallCpuLoop = CallInst::Create(CpuLoop, { EnvPtr }, "", Call);

    // In recent versions of LLVM you can no longer inject a CallInst in a
    // Function with debug location if the call itself has not a debug location
    // as well, otherwise verifyModule() will fail.
    CallCpuLoop->setDebugLoc(Call->getDebugLoc());

    // Set cpu_loop_exiting to true
    new StoreInst(ConstantInt::getTrue(BoolType), CpuLoopExitingVariable, Call);

    // Return immediately
    createRet(Call);
    auto *Unreach = cast<UnreachableInst>(&*++Call->getIterator());
    Unreach->eraseFromParent();

    Function *Caller = Call->getParent()->getParent();

    // Remove the call to cpu_loop_exit
    Call->eraseFromParent();

    if (FixedCallers.find(Caller) == FixedCallers.end()) {
      FixedCallers.insert(Caller);

      std::queue<Value *> WorkList;
      WorkList.push(Caller);

      while (!WorkList.empty()) {
        Value *F = WorkList.front();
        WorkList.pop();

        for (User *RecUser : F->users()) {
          auto *RecCall = dyn_cast<CallInst>(RecUser);
          if (RecCall == nullptr) {
            auto *Cast = dyn_cast<ConstantExpr>(RecUser);
            revng_assert(Cast != nullptr, "Unexpected user");
            revng_assert(Cast->getOperand(0) == F && Cast->isCast());
            WorkList.push(Cast);
            continue;
          }

          Function *RecCaller = RecCall->getParent()->getParent();

          // TODO: make this more reliable than using function name
          // If the caller is a QEMU helper function make it check
          // cpu_loop_exiting and if it's true, make it return

          // Split BB
          BasicBlock *OldBB = RecCall->getParent();
          BasicBlock::iterator SplitPoint = ++RecCall->getIterator();
          revng_assert(SplitPoint != OldBB->end());
          BasicBlock *NewBB = OldBB->splitBasicBlock(SplitPoint);

          // Add a BB with a ret
          BasicBlock *QuitBB = BasicBlock::Create(Context,
                                                  "cpu_loop_exit_return",
                                                  RecCaller,
                                                  NewBB);
          UnreachableInst *Temp = new UnreachableInst(Context, QuitBB);
          createRet(Temp);
          Temp->eraseFromParent();

          // Check value of cpu_loop_exiting
          auto *Branch = cast<BranchInst>(&*++(RecCall->getIterator()));
          auto *CPULoopExitVarPtrTy = CpuLoopExitingVariable->getType();
          auto *PointeeTy = CPULoopExitVarPtrTy->getPointerElementType();
          auto *Compare = new ICmpInst(Branch,
                                       CmpInst::ICMP_EQ,
                                       new LoadInst(PointeeTy,
                                                    CpuLoopExitingVariable,
                                                    "",
                                                    Branch),
                                       ConstantInt::getTrue(BoolType));

          BranchInst::Create(QuitBB, NewBB, Compare, Branch);
          Branch->eraseFromParent();

          // Add to the work list only if it hasn't been fixed already
          if (FixedCallers.find(RecCaller) == FixedCallers.end()) {
            FixedCallers.insert(RecCaller);
            WorkList.push(RecCaller);
          }
        }
      }
    }
  }

  return true;
}

void CodeGenerator::translate(Optional<uint64_t> RawVirtualAddress) {
  using FT = FunctionType;

  // Declare the abort function
  auto *AbortTy = FunctionType::get(Type::getVoidTy(Context), false);
  FunctionCallee AbortFunction = TheModule->getOrInsertFunction("abort",
                                                                AbortTy);
  {
    auto *Abort = cast<Function>(skipCasts(AbortFunction.getCallee()));
    FunctionTags::Exceptional.addTo(Abort);
  }

  // Prepare the helper modules by transforming the cpu_loop function and
  // running SROA
  legacy::PassManager CpuLoopPM;
  CpuLoopPM.add(new LoopInfoWrapperPass());
  CpuLoopPM.add(new CpuLoopFunctionPass(ptc.exception_index));
  CpuLoopPM.add(createSROAPass());
  CpuLoopPM.run(*HelpersModule);

  // Drop the main
  HelpersModule->getFunction("main")->eraseFromParent();

  // From syscall.c
  new GlobalVariable(*TheModule,
                     Type::getInt32Ty(Context),
                     false,
                     GlobalValue::CommonLinkage,
                     ConstantInt::get(Type::getInt32Ty(Context), 0),
                     StringRef("do_strace"));

  //
  // Handle some specific QEMU functions as no-ops or abort
  //

  // Transform in no op
  auto NoOpFunctionNames = make_array<const char *>("cpu_dump_state",
                                                    "cpu_exit",
                                                    "end_exclusive"
                                                    "fprintf",
                                                    "mmap_lock",
                                                    "mmap_unlock",
                                                    "pthread_cond_broadcast",
                                                    "pthread_mutex_unlock",
                                                    "pthread_mutex_lock",
                                                    "pthread_cond_wait",
                                                    "pthread_cond_signal",
                                                    "process_pending_signals",
                                                    "qemu_log_mask",
                                                    "qemu_thread_atexit_init",
                                                    "start_exclusive");
  for (auto Name : NoOpFunctionNames)
    replaceFunctionWithRet(HelpersModule->getFunction(Name), 0);

  // Transform in abort

  // do_arm_semihosting: we don't care about semihosting
  // EmulateAll: requires access to the opcode
  auto AbortFunctionNames = make_array<const char *>("cpu_restore_state",
                                                     "cpu_mips_exec",
                                                     "gdb_handlesig",
                                                     "queue_signal",
                                                     // syscall.c
                                                     "do_ioctl_dm",
                                                     "print_syscall",
                                                     "print_syscall_ret",
                                                     // ARM cpu_loop
                                                     "cpu_abort",
                                                     "do_arm_semihosting",
                                                     "EmulateAll");
  for (auto Name : AbortFunctionNames) {
    Function *TheFunction = HelpersModule->getFunction(Name);
    if (TheFunction != nullptr) {
      revng_assert(HelpersModule->getFunction("abort") != nullptr);
      BasicBlock *NewBody = replaceFunction(TheFunction);
      CallInst::Create(HelpersModule->getFunction("abort"), {}, NewBody);
      new UnreachableInst(Context, NewBody);
    }
  }

  replaceFunctionWithRet(HelpersModule->getFunction("page_check_range"), 1);
  replaceFunctionWithRet(HelpersModule->getFunction("page_get_flags"),
                         0xffffffff);

  //
  // Record globals for marking them as internal after linking
  //
  std::vector<std::string> HelperGlobals;
  for (GlobalVariable &GV : HelpersModule->globals())
    if (GV.hasName())
      HelperGlobals.push_back(GV.getName().str());

  std::vector<std::string> HelperFunctions;
  for (Function &F : HelpersModule->functions())
    if (F.hasName() and F.getName() != "target_set_brk"
        and F.getName() != "syscall_init")
      HelperFunctions.push_back(F.getName().str());

  //
  // Link helpers module into the main module
  //
  Linker TheLinker(*TheModule);
  bool Result = TheLinker.linkInModule(std::move(HelpersModule));
  revng_assert(not Result, "Linking failed");

  //
  // Mark as internal all the imported globals
  //
  for (StringRef GlobalName : HelperGlobals)
    if (not GlobalName.startswith("llvm."))
      if (auto *GV = TheModule->getGlobalVariable(GlobalName))
        if (not GV->isDeclaration())
          GV->setLinkage(GlobalValue::InternalLinkage);

  for (StringRef FunctionName : HelperFunctions)
    if (auto *F = TheModule->getFunction(FunctionName))
      if (not F->isDeclaration() and not F->isIntrinsic())
        F->setLinkage(GlobalValue::InternalLinkage);

  //
  // Create the VariableManager
  //
  VariableManager Variables(*TheModule, TargetArchitecture);
  auto createCPUStateAccessAnalysisPass = [&Variables]() {
    return new CPUStateAccessAnalysisPass(&Variables, true);
  };

  {
    legacy::PassManager PM;
    PM.add(new CpuLoopExitPass(&Variables));
    PM.run(*TheModule);
  }

  std::set<Function *> CpuLoopExitingUsers;
  Value *CpuLoopExiting = TheModule->getGlobalVariable("cpu_loop_exiting");
  revng_assert(CpuLoopExiting != nullptr);
  for (User *U : CpuLoopExiting->users())
    if (auto *I = dyn_cast<Instruction>(U))
      CpuLoopExitingUsers.insert(I->getParent()->getParent());

  //
  // Create well-known CSVs
  //
  const Architecture &Arch = Binary.architecture();
  std::string SPName = Arch.stackPointerRegister().str();
  GlobalVariable *SPReg = Variables.getByEnvOffset(ptc.sp, SPName).first;

  using PCHOwner = std::unique_ptr<ProgramCounterHandler>;
  auto Factory = [&Variables](PCAffectingCSV::Values CSVID,
                              llvm::StringRef Name) -> GlobalVariable * {
    intptr_t Offset = 0;

    switch (CSVID) {
    case PCAffectingCSV::PC:
      Offset = ptc.pc;
      break;

    case PCAffectingCSV::IsThumb:
      Offset = ptc.is_thumb;
      break;

    default:
      revng_abort();
    }

    return Variables.getByEnvOffset(Offset, Name.str()).first;
  };
  PCHOwner PCH = ProgramCounterHandler::create(Arch.type(),
                                               TheModule.get(),
                                               Factory);

  IRBuilder<> Builder(Context);

  // Create main function
  auto *MainType = FT::get(Builder.getVoidTy(),
                           { SPReg->getType()->getPointerElementType() },
                           false);
  auto *MainFunction = Function::Create(MainType,
                                        Function::ExternalLinkage,
                                        "root",
                                        TheModule.get());
  FunctionTags::Root.addTo(MainFunction);

  // Create the first basic block and create a placeholder for variable
  // allocations
  BasicBlock *Entry = BasicBlock::Create(Context, "entrypoint", MainFunction);
  Builder.SetInsertPoint(Entry);

  // We need to remember this instruction so we can later insert a call here.
  // The problem is that up until now we don't know where our CPUState structure
  // is.
  // After the translation we will and use this information to create a call to
  // a helper function.
  // TODO: we need a more elegant solution here
  auto *Delimiter = Builder.CreateStore(&*MainFunction->arg_begin(), SPReg);
  Variables.setAllocaInsertPoint(Delimiter);
  auto *InitEnvInsertPoint = Delimiter;

  QuickMetadata QMD(Context);

  //
  // Create revng.input named metadata
  //

  const char *MDName = "revng.input.canonical-values";
  NamedMDNode *CanonicalValuesMD;
  CanonicalValuesMD = TheModule->getOrInsertNamedMetadata(MDName);
  for (auto &P : Binary.canonicalValues()) {
    StringRef CSVName = P.first;
    uint64_t CanonicalValue = P.second;
    ArrayRef<Metadata *> Entry{ QMD.get(CSVName), QMD.get(CanonicalValue) };
    CanonicalValuesMD->addOperand(QMD.tuple(Entry));
  }

  //
  // Currently revng.input.architecture is composed as follows:
  //
  // revng.input.architecture = {
  //   ArchitectureName,
  //   InstructionAlignment,
  //   DelaySlotSize,
  //   PCRegisterName,
  //   SPRegisterName,
  //   ABIRegisters
  // }

  const SmallVector<ABIRegister, 20> &ABIRegisters = Arch.abiRegisters();
  SmallVector<Metadata *, 20> ABIRegMetadata;
  for (auto Register : ABIRegisters)
    ABIRegMetadata.push_back(MDString::get(Context, Register.csvName()));

  auto *Tuple = MDTuple::get(Context,
                             {
                               QMD.get(Arch.name()),
                               QMD.get(Arch.instructionAlignment()),
                               QMD.get(Arch.delaySlotSize()),
                               QMD.get("pc"),
                               QMD.get(Arch.stackPointerRegister()),
                               QMD.tuple(ArrayRef<Metadata *>(ABIRegMetadata)),
                             });
  MDName = "revng.input.architecture";
  NamedMDNode *InputArchMD = TheModule->getOrInsertNamedMetadata(MDName);
  InputArchMD->addOperand(Tuple);

  // Link early-linked.c
  {
    Linker TheLinker(*TheModule);
    bool Result = TheLinker.linkInModule(std::move(EarlyLinkedModule),
                                         Linker::None);
    revng_assert(!Result, "Linking failed");
  }

  // Create an instance of JumpTargetManager
  JumpTargetManager JumpTargets(MainFunction,
                                PCH.get(),
                                Binary,
                                createCPUStateAccessAnalysisPass);

  MetaAddress VirtualAddress = MetaAddress::invalid();
  if (RawVirtualAddress) {
    VirtualAddress = JumpTargets.fromPC(*RawVirtualAddress);
  } else {
    JumpTargets.harvestGlobalData();
    VirtualAddress = Binary.entryPoint();
    revng_assert(VirtualAddress.isCode());
  }

  if (VirtualAddress.isValid()) {
    JumpTargets.registerJT(VirtualAddress, JTReason::GlobalData);

    // Initialize the program counter
    PCH->initializePC(Builder, VirtualAddress);
  }

  OpaqueIdentity OI(TheModule.get());

  // Fake jumps to the dispatcher-related basic blocks. This way all the blocks
  // are always reachable.
  auto *ReachSwitch = Builder.CreateSwitch(OI.wrap(Builder, Builder.getInt8(0)),
                                           JumpTargets.dispatcher());
  ReachSwitch->addCase(Builder.getInt8(1), JumpTargets.anyPC());
  ReachSwitch->addCase(Builder.getInt8(2), JumpTargets.unexpectedPC());

  JumpTargets.setCFGForm(CFGForm::SemanticPreserving);

  std::tie(VirtualAddress, Entry) = JumpTargets.peek();

  std::vector<BasicBlock *> Blocks;

  InstructionTranslator Translator(Builder,
                                   Variables,
                                   JumpTargets,
                                   Blocks,
                                   Binary.architecture(),
                                   TargetArchitecture,
                                   PCH.get());

  while (Entry != nullptr) {
    Builder.SetInsertPoint(Entry);

    // TODO: what if create a new instance of an InstructionTranslator here?
    Translator.reset();

    // TODO: rename this type
    PTCInstructionListPtr InstructionList(new PTCInstructionList);
    size_t ConsumedSize = 0;

    PTCCodeType Type = PTC_CODE_REGULAR;

    switch (VirtualAddress.type()) {
    case MetaAddressType::Invalid:
      revng_abort();

    case MetaAddressType::Code_arm_thumb:
      Type = PTC_CODE_ARM_THUMB;
      break;

    default:
      Type = PTC_CODE_REGULAR;
      break;
    }

    ConsumedSize = ptc.translate(VirtualAddress.address(),
                                 Type,
                                 InstructionList.get());

    // Check whether we ended up in an unmapped page
    MetaAddress AbortAt = MetaAddress::invalid();
    MetaAddress LastByte = VirtualAddress.toGeneric() + (ConsumedSize - 1);
    if (VirtualAddress.pageStart() != LastByte.pageStart()) {
      MetaAddress NextPage = VirtualAddress.nextPageStart();
      if (NoMoreCodeBoundaries.count(NextPage) != 0)
        AbortAt = NextPage;
    }

    SmallSet<unsigned, 1> ToIgnore;
    ToIgnore = Translator.preprocess(InstructionList.get());

    if (PTCLog.isEnabled()) {
      std::stringstream Stream;
      dumpTranslation(VirtualAddress, Stream, InstructionList.get());
      PTCLog << Stream.str() << DoLog;
    }

    Variables.newFunction(InstructionList.get());
    unsigned j = 0;
    MDNode *MDOriginalInstr = nullptr;
    bool StopTranslation = false;

    MetaAddress PC = VirtualAddress;
    MetaAddress NextPC = MetaAddress::invalid();
    MetaAddress EndPC = VirtualAddress + ConsumedSize;

    const auto InstructionCount = InstructionList->instruction_count;
    using IT = InstructionTranslator;
    IT::TranslationResult Result;

    // Handle the first PTC_INSTRUCTION_op_debug_insn_start
    {
      PTCInstruction *NextInstruction = nullptr;
      for (unsigned k = 1; k < InstructionCount; k++) {
        PTCInstruction *I = &InstructionList->instructions[k];
        if (I->opc == PTC_INSTRUCTION_op_debug_insn_start
            && ToIgnore.count(k) == 0) {
          NextInstruction = I;
          break;
        }
      }
      PTCInstruction *Instruction = &InstructionList->instructions[j];
      std::tie(Result,
               MDOriginalInstr,
               PC,
               NextPC) = Translator.newInstruction(Instruction,
                                                   NextInstruction,
                                                   VirtualAddress,
                                                   EndPC,
                                                   true,
                                                   AbortAt);
      j++;
    }

    // TODO: shall we move this whole loop in InstructionTranslator?
    for (; j < InstructionCount && !StopTranslation; j++) {
      if (ToIgnore.count(j) != 0)
        continue;

      PTCInstruction Instruction = InstructionList->instructions[j];
      PTCOpcode Opcode = Instruction.opc;

      Blocks.clear();
      Blocks.push_back(Builder.GetInsertBlock());

      switch (Opcode) {
      case PTC_INSTRUCTION_op_discard:
        // Instructions we don't even consider
        break;
      case PTC_INSTRUCTION_op_debug_insn_start: {
        // Find next instruction, if there is one
        PTCInstruction *NextInstruction = nullptr;
        for (unsigned k = j + 1; k < InstructionCount; k++) {
          PTCInstruction *I = &InstructionList->instructions[k];
          if (I->opc == PTC_INSTRUCTION_op_debug_insn_start
              && ToIgnore.count(k) == 0) {
            NextInstruction = I;
            break;
          }
        }

        std::tie(Result,
                 MDOriginalInstr,
                 PC,
                 NextPC) = Translator.newInstruction(&Instruction,
                                                     NextInstruction,
                                                     VirtualAddress,
                                                     EndPC,
                                                     false,
                                                     AbortAt);
      } break;
      case PTC_INSTRUCTION_op_call: {
        Result = Translator.translateCall(&Instruction);

        // Sometimes libtinycode terminates a basic block with a call, in this
        // case force a fallthrough
        auto &IL = InstructionList;
        if (j == IL->instruction_count - 1) {
          BasicBlock *Target = JumpTargets.registerJT(EndPC,
                                                      JTReason::PostHelper);
          Builder.CreateBr(notNull(Target));
        }

      } break;

      default:
        Result = Translator.translate(&Instruction, PC, NextPC);
        break;
      }

      switch (Result) {
      case IT::Success:
        // No-op
        break;
      case IT::Abort:
        Builder.CreateCall(AbortFunction);
        Builder.CreateUnreachable();
        StopTranslation = true;
        break;
      case IT::Stop:
        StopTranslation = true;
        break;
      }

      // Create a new metadata referencing the PTC instruction we have just
      // translated
      MDNode *MDPTCInstr = nullptr;
      if (RecordPTC) {
        std::stringstream PTCStringStream;
        dumpInstruction(PTCStringStream, InstructionList.get(), j);
        std::string PTCString = PTCStringStream.str() + "\n";
        MDString *MDPTCString = MDString::get(Context, PTCString);
        MDPTCInstr = MDNode::getDistinct(Context, MDPTCString);
      }

      // Set metadata for all the new instructions
      for (BasicBlock *Block : Blocks) {
        BasicBlock::iterator I = Block->end();
        while (I != Block->begin() && !(--I)->hasMetadata()) {
          if (MDOriginalInstr != nullptr)
            I->setMetadata(OriginalInstrMDKind, MDOriginalInstr);
          if (MDPTCInstr != nullptr)
            I->setMetadata(PTCInstrMDKind, MDPTCInstr);
        }
      }

    } // End loop over instructions

    // We might have a leftover block, probably due to the block created after
    // the last call to exit_tb
    auto *LastBlock = Builder.GetInsertBlock();
    if (LastBlock->empty())
      LastBlock->eraseFromParent();
    else if (!LastBlock->rbegin()->isTerminator()) {
      // Something went wrong, probably a mistranslation
      Builder.CreateUnreachable();
    }

    Translator.registerDirectJumps();

    // Obtain a new program counter to translate
    std::tie(VirtualAddress, Entry) = JumpTargets.peek();
  } // End translations loop

  OI.drop();

  // Reorder basic blocks in RPOT
  {
    BasicBlock *Entry = &MainFunction->getEntryBlock();
    ReversePostOrderTraversal<BasicBlock *> RPOT(Entry);
    std::set<BasicBlock *> SortedBasicBlocksSet;
    std::vector<BasicBlock *> SortedBasicBlocks;
    for (BasicBlock *BB : RPOT) {
      SortedBasicBlocksSet.insert(BB);
      SortedBasicBlocks.push_back(BB);
    }

    auto &BasicBlockList = MainFunction->getBasicBlockList();
    std::vector<BasicBlock *> Unreachable;
    for (BasicBlock &BB : BasicBlockList) {
      if (SortedBasicBlocksSet.count(&BB) == 0) {
        Unreachable.push_back(&BB);
      }
    }

    auto Size = BasicBlockList.size();
    for (unsigned I = 0; I < Size; ++I)
      BasicBlockList.begin()->removeFromParent();
    for (BasicBlock *BB : SortedBasicBlocks)
      BasicBlockList.push_back(BB);
    for (BasicBlock *BB : Unreachable)
      BasicBlockList.push_back(BB);
  }

  //
  // At this point we have all the code, add store false to cpu_loop_exiting in
  // root
  //
  auto *BoolType = CpuLoopExiting->getType()->getPointerElementType();
  std::queue<User *> WorkList;
  for (Function *Helper : CpuLoopExitingUsers)
    for (User *U : Helper->users())
      WorkList.push(U);

  while (not WorkList.empty()) {
    User *U = WorkList.front();
    WorkList.pop();

    if (auto *CE = dyn_cast<ConstantExpr>(U)) {
      if (CE->isCast())
        for (User *UCE : CE->users())
          WorkList.push(UCE);
    } else if (auto *Call = dyn_cast<CallInst>(U)) {
      if (Call->getParent()->getParent() == MainFunction) {
        new StoreInst(ConstantInt::getFalse(BoolType),
                      CpuLoopExiting,
                      Call->getNextNode());
      }
    }
  }

  // Add a call to the function to initialize the CPUState, if present.
  // This is important on x86 architecture.
  // We only add the call after the Linker has imported the
  // initialize_env function from the helpers, because the declaration
  // imported before with importHelperFunctionDeclaration() only has
  // stub types and injecting the CallInst earlier would break
  if (Function *InitEnv = TheModule->getFunction("initialize_env")) {
    revng_assert(not InitEnv->getFunctionType()->isVarArg());
    revng_assert(InitEnv->getFunctionType()->getNumParams() == 1);
    auto *CPUStateType = InitEnv->getFunctionType()->getParamType(0);
    Instruction *InsertBefore = InitEnvInsertPoint;
    auto *AddressComputation = Variables.computeEnvAddress(CPUStateType,
                                                           InsertBefore);
    CallInst::Create(InitEnv, { AddressComputation }, "", InsertBefore);
  }

  Variables.setDataLayout(&TheModule->getDataLayout());

  Translator.finalizeNewPCMarkers(CoveragePath);

  // SROA must run before InstCombine because in this way InstCombine has many
  // more elementary operations to combine
  legacy::PassManager PreInstCombinePM;
  PreInstCombinePM.add(createSROAPass());
  PreInstCombinePM.run(*TheModule);

  // InstCombine must run before CPUStateAccessAnalysis (CSAA) because, if it
  // runs after it, it removes all the useful metadata attached by CSAA.
  legacy::FunctionPassManager InstCombinePM(&*TheModule);
  InstCombinePM.add(createInstructionCombiningPass());
  InstCombinePM.doInitialization();
  InstCombinePM.run(*MainFunction);
  InstCombinePM.doFinalization();

  legacy::PassManager PostInstCombinePM;
  PostInstCombinePM.add(new CPUStateAccessAnalysisPass(&Variables, false));
  PostInstCombinePM.add(createDeadCodeEliminationPass());
  PostInstCombinePM.add(new PruneRetSuccessors);
  PostInstCombinePM.run(*TheModule);

  // Serialize an empty Model into TheModule
  model::Binary Model;
  writeModel(Model, *TheModule);

  JumpTargets.finalizeJumpTargets();

  EliminateUnreachableBlocks(*MainFunction, nullptr, false);

  JumpTargets.createJTReasonMD();

  ExternalJumpsHandler JumpOutHandler(Binary,
                                      JumpTargets.dispatcher(),
                                      *MainFunction,
                                      PCH.get());
  JumpOutHandler.createExternalJumpsHandler();

  Variables.finalize();

  Debug->generateDebugInfo();
}

void CodeGenerator::serialize() {
  // Ask the debug handler if it already has a good copy of the IR, if not dump
  // it
  if (!Debug->copySource()) {
    std::ofstream Output(OutputPath);
    Debug->print(Output, false);
  }
}
