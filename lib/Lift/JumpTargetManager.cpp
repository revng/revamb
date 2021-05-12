/// \file JumpTargetManager.cpp
/// \brief This file handles the possible jump targets encountered during
///        translation and the creation and management of the respective
///        BasicBlock.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <fstream>
#include <queue>
#include <sstream>

#include "boost/icl/interval_set.hpp"
#include "boost/icl/right_open_interval.hpp"
#include "boost/type_traits/is_same.hpp"

#include "llvm/ADT/DepthFirstIterator.h"
#include "llvm/Analysis/BasicAliasAnalysis.h"
#include "llvm/Analysis/ScopedNoAliasAA.h"
#include "llvm/CodeGen/UnreachableBlockElim.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/MDBuilder.h"
#include "llvm/IR/Verifier.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/EarlyCSE.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include "llvm/Transforms/Scalar/JumpThreading.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/Transforms/Utils/Local.h"
#include "llvm/Transforms/Utils/Mem2Reg.h"

#include "revng/ADT/Queue.h"
#include "revng/BasicAnalyses/AdvancedValueInfo.h"
#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/BasicAnalyses/ShrinkInstructionOperandsPass.h"
#include "revng/FunctionCallIdentification/FunctionCallIdentification.h"
#include "revng/Lift/AdvancedValueInfoPass.h"
#include "revng/Lift/CPUStateAccessAnalysisPass.h"
#include "revng/Lift/DropHelperCallsPass.h"
#include "revng/Lift/JumpTargetManager.h"
#include "revng/Lift/SubGraph.h"
#include "revng/Support/Assert.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/Debug.h"
#include "revng/Support/FunctionTags.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/MetaAddress.h"
#include "revng/Support/Statistics.h"
#include "revng/Support/revng.h"
#include "revng/TypeShrinking/BitLiveness.h"
#include "revng/TypeShrinking/TypeShrinking.h"

#include "AdvancedValueInfoPass.h"
#include "CPUStateAccessAnalysisPass.h"
#include "DropHelperCallsPass.h"
#include "JumpTargetManager.h"
#include "SubGraph.h"

using namespace llvm;

namespace {

Logger<> JTCountLog("jtcount");
Logger<> NewEdgesLog("new-edges");
Logger<> RegisterJTLog("registerjt");

CounterMap<std::string> HarvestingStats("harvesting");
RunningStatistics BlocksAnalyzedByAVI("blocks-analyzed-by-avi");

RegisterPass<TranslateDirectBranchesPass> X("translate-db",
                                            "Translate Direct Branches"
                                            " Pass",
                                            false,
                                            false);

} // namespace

char TranslateDirectBranchesPass::ID = 0;

void TranslateDirectBranchesPass::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<DominatorTreeWrapperPass>();
  AU.setPreservesAll();
}

void JumpTargetManager::assertNoUnreachable() const {
  std::set<BasicBlock *> Unreachable = computeUnreachable();
  if (Unreachable.size() != 0) {
    VerifyLog << "The following basic blocks are unreachable:\n";
    for (BasicBlock *BB : Unreachable) {
      VerifyLog << "  " << getName(BB) << " (predecessors:";
      for (BasicBlock *Predecessor : make_range(pred_begin(BB), pred_end(BB)))
        VerifyLog << " " << getName(Predecessor);

      MetaAddress PC = getBasicBlockPC(BB);
      if (PC.isValid()) {
        auto It = JumpTargets.find(PC);
        if (It != JumpTargets.end()) {
          VerifyLog << ", reasons:";
          for (const char *Reason : It->second.getReasonNames())
            VerifyLog << " " << Reason;
        }
      }

      VerifyLog << ")\n";
    }
    VerifyLog << DoLog;
    revng_abort();
  }
}

/// \brief Purges everything is after a call to exitTB (except the call itself)
static void exitTBCleanup(Instruction *ExitTBCall) {
  BasicBlock *BB = ExitTBCall->getParent();

  // Cleanup everything it's aftewards starting from the end
  Instruction *ToDelete = &*(--BB->end());
  while (ToDelete != ExitTBCall) {
    if (auto DeadBranch = dyn_cast<BranchInst>(ToDelete))
      purgeBranch(BasicBlock::iterator(DeadBranch));
    else
      ToDelete->eraseFromParent();

    ToDelete = &*(--BB->end());
  }
}

using TDBP = TranslateDirectBranchesPass;

TDBP::TranslateDirectBranchesPass(JumpTargetManager *J) :
  ModulePass(ID), JTM(J), PCH(J->programCounterHandler()) {
}

using DispatcherTargets = ProgramCounterHandler::DispatcherTargets;
void TDBP::pinExitTB(CallInst *ExitTBCall, DispatcherTargets &Destinations) {
  revng_assert(ExitTBCall != nullptr);
  revng_assert(Destinations.size() != 0);

  LLVMContext &Context = getContext(ExitTBCall);
  BasicBlock *AnyPC = JTM->anyPC();
  BasicBlock *UnexpectedPC = JTM->unexpectedPC();
  BasicBlock *Source = ExitTBCall->getParent();

  using CI = ConstantInt;
  auto *ExitTBArg = CI::get(Type::getInt32Ty(Context), Destinations.size());
  uint64_t OldTargetsCount = getLimitedValue(ExitTBCall->getArgOperand(0));

  // TODO: we should check Destinations.size() >= OldTargetsCount
  // TODO: we should also check the destinations are actually the same

  BasicBlock *BB = ExitTBCall->getParent();

  if (auto *Dispatcher = dyn_cast<SwitchInst>(BB->getTerminator()))
    PCH->destroyDispatcher(Dispatcher);

  // Kill everything is after the call to exitTB
  exitTBCleanup(ExitTBCall);

  // Mark this call to exitTB as handled
  ExitTBCall->setArgOperand(0, ExitTBArg);

  PCH->buildDispatcher(Destinations,
                       BB,
                       UnexpectedPC,
                       BlockType::IndirectBranchDispatcherHelperBlock);

  // Move all the markers right before the branch instruction
  Instruction *Last = BB->getTerminator();
  auto It = ExitTBCall->getIterator();
  while (isMarker(&*It)) {
    // Get the marker instructions
    Instruction *I = &*It;

    // Move the iterator back
    It--;

    // Move the last moved instruction (initially the terminator)
    I->moveBefore(Last);

    Last = I;
  }

  // Notify new branches only if the amount of possible targets actually
  // increased
  if (Destinations.size() > OldTargetsCount)
    JTM->recordNewBranches(Source, Destinations.size() - OldTargetsCount);
}

bool TDBP::pinAVIResults(Function &F) {
  QuickMetadata QMD(getContext(&F));

  Function *ExitTB = JTM->exitTB();
  for (User *U : ExitTB->users()) {
    if (auto *Call = dyn_cast<CallInst>(U)) {
      if (Call->getParent()->getParent() != &F)
        continue;

      auto *MD = Call->getMetadata("revng.targets");
      if (auto *T = dyn_cast_or_null<MDTuple>(MD)) {
        if (T->getNumOperands() > 0) {
          // We have at least a value, prepare a list of jump targets
          ProgramCounterHandler::DispatcherTargets Values;
          Values.reserve(T->getNumOperands());
          for (const MDOperand &Operand : T->operands()) {
            auto *CMA = QMD.extract<Constant *>(Operand.get());
            auto MA = MetaAddress::fromConstant(CMA);
            Values.emplace_back(MA, JTM->getBlockAt(MA));
          }

          pinExitTB(Call, Values);
        }
      }
    }
  }

  return true;
}

void TDBP::pinConstantStoreInternal(MetaAddress Address, CallInst *ExitTBCall) {
  revng_assert(Address.isValid());

  BasicBlock *TargetBlock = JTM->registerJT(Address, JTReason::DirectJump);

  const Module *M = getModule(ExitTBCall);
  LLVMContext &Context = getContext(M);

  // Remove unreachable right after the exit_tb
  BasicBlock::iterator CallIt(ExitTBCall);
  BasicBlock::iterator BlockEnd = ExitTBCall->getParent()->end();
  CallIt++;
  revng_assert(CallIt != BlockEnd and isa<UnreachableInst>(&*CallIt));
  CallIt->eraseFromParent();

  // Cleanup of what's afterwards (only a unconditional jump is
  // allowed)
  CallIt = BasicBlock::iterator(ExitTBCall);
  BlockEnd = ExitTBCall->getParent()->end();
  if (++CallIt != BlockEnd)
    purgeBranch(CallIt);

  if (TargetBlock != nullptr) {
    // A target was found, jump there
    BranchInst::Create(TargetBlock, ExitTBCall->getParent());
    JTM->recordNewBranches(ExitTBCall->getParent(), 1);
  } else {
    // We're jumping to an unknown or invalid location,
    // jump back to the dispatcher
    // TODO: emit a warning
    BranchInst::Create(JTM->unexpectedPC(), ExitTBCall);
  }

  ExitTBCall->eraseFromParent();
}

bool TDBP::pinConstantStore(Function &F) {
  auto ExitTB = JTM->exitTB();
  auto ExitTBIt = ExitTB->use_begin();
  while (ExitTBIt != ExitTB->use_end()) {
    // Take note of the use and increment the iterator immediately: this allows
    // us to erase the call to exit_tb without unexpected behaviors
    Use &ExitTBUse = *ExitTBIt++;
    auto *Call = cast<CallInst>(ExitTBUse.getUser());
    revng_assert(Call->getCalledFunction() == ExitTB);

    // Look for the last write to the PC
    auto [Result, NextPC] = PCH->getUniqueJumpTarget(Call->getParent());

    switch (Result) {
    case NextJumpTarget::Unique:
      // A constant store was born
      revng_assert(NextPC.isValid());
      pinConstantStoreInternal(NextPC, Call);
      break;

    case NextJumpTarget::Multiple:
      // Nothing to do, it's an indirect jump
      break;

    case NextJumpTarget::Helper:
      forceFallthroughAfterHelper(Call);
      break;

    default:
      revng_abort();
    }
  }

  return true;
}

bool TDBP::forceFallthroughAfterHelper(CallInst *Call) {
  // If someone else already took care of the situation, quit
  if (getLimitedValue(Call->getArgOperand(0)) > 0)
    return false;

  bool ForceFallthrough = false;

  BasicBlock::reverse_iterator It(++Call->getReverseIterator());
  auto *BB = Call->getParent();
  auto EndIt = BB->rend();
  while (!ForceFallthrough) {
    while (It != EndIt) {
      Instruction *I = &*It;
      if (auto *Store = dyn_cast<StoreInst>(I)) {
        if (PCH->affectsPC(Store)) {
          // We found a PC-store, give up
          return false;
        }
      } else if (isCallToHelper(I)) {
        // We found a call to an helper
        ForceFallthrough = true;
        break;
      }
      It++;
    }

    if (!ForceFallthrough) {
      // Proceed only to unique predecessor, if present
      if (auto *Pred = BB->getUniquePredecessor()) {
        BB = Pred;
        It = BB->rbegin();
        EndIt = BB->rend();
      } else {
        // We have multiple predecessors, give up
        return false;
      }
    }
  }

  exitTBCleanup(Call);

  IRBuilder<> Builder(Call->getParent());
  Call->setArgOperand(0, Builder.getInt32(1));

  // Create the fallthrough jump
  MetaAddress NextPC = JTM->getNextPC(Call);

  // Get the fallthrough basic block and emit a conditional branch, if not
  // possible simply jump to anyPC
  BasicBlock *AnyPC = JTM->anyPC();
  if (BasicBlock *NextPCBB = JTM->registerJT(NextPC, JTReason::PostHelper)) {
    PCH->buildHotPath(Builder, { NextPC, NextPCBB }, AnyPC);
  } else {
    Builder.CreateBr(AnyPC);
  }

  JTM->recordNewBranches(Call->getParent(), 1);

  return true;
}

bool TDBP::runOnModule(Module &M) {
  Function &F = *M.getFunction("root");
  pinConstantStore(F);
  pinAVIResults(F);
  return true;
}

MaterializedValue
JumpTargetManager::readFromPointer(Constant *Pointer, BinaryFile::Endianess E) {
  Type *LoadedType = Pointer->getType()->getPointerElementType();
  const DataLayout &DL = TheModule.getDataLayout();
  unsigned LoadSize = DL.getTypeSizeInBits(LoadedType) / 8;
  auto NewAPInt = [LoadSize](uint64_t V) { return APInt(LoadSize * 8, V); };

  Value *RealPointer = skipCasts(Pointer);
  uint64_t RawLoadAddress = 0;
  if (not isa<ConstantPointerNull>(RealPointer)) {
    RawLoadAddress = getZExtValue(cast<ConstantInt>(RealPointer), DL);
  }
  auto LoadAddress = fromAbsolute(RawLoadAddress);

  UnusedCodePointers.erase(LoadAddress);
  registerReadRange(LoadAddress, LoadSize);

  // Prevent overflow when computing the label interval
  if ((LoadAddress + LoadSize).addressLowerThan(LoadAddress)) {
    return MaterializedValue::invalid();
  }

  const auto &Labels = binary().labels();
  using interval = boost::icl::interval<MetaAddress, compareAddress>;
  auto Interval = interval::right_open(LoadAddress, LoadAddress + LoadSize);
  auto It = Labels.find(Interval);
  if (It != Labels.end()) {
    const Label *Match = nullptr;
    for (const Label *Candidate : It->second) {
      if (Candidate->size() == LoadSize
          and (Candidate->isAbsoluteValue() or Candidate->isBaseRelativeValue()
               or Candidate->isSymbolRelativeValue())) {

        if (Match != nullptr
            and (Match->symbolName() != Candidate->symbolName()
                 or Match->offset() != Candidate->offset())) {
          revng_abort("Multiple value labels at the same location");
        }

        Match = Candidate;
      }
    }

    if (Match != nullptr) {
      switch (Match->type()) {
      case LabelType::AbsoluteValue:
        return { NewAPInt(Match->value()) };

      case LabelType::BaseRelativeValue:
        return { NewAPInt(binary().relocate(Match->value()).address()) };

      case LabelType::SymbolRelativeValue:
        return { Match->symbolName(), NewAPInt(Match->offset()) };

      default:
        revng_abort();
      }
    }
  }

  // No labels found, fall back to read the raw value, if available
  Optional<uint64_t> Value = Binary.readRawValue(LoadAddress, LoadSize, E);

  if (Value)
    return { NewAPInt(*Value) };
  else
    return {};
}

template<typename T>
static cl::opt<T> *
getOption(StringMap<cl::Option *> &Options, const char *Name) {
  return static_cast<cl::opt<T> *>(Options[Name]);
}

JumpTargetManager::JumpTargetManager(Function *TheFunction,
                                     ProgramCounterHandler *PCH,
                                     const BinaryFile &Binary,
                                     CSAAFactory createCSAA) :
  TheModule(*TheFunction->getParent()),
  Context(TheModule.getContext()),
  TheFunction(TheFunction),
  OriginalInstructionAddresses(),
  JumpTargets(),
  ExitTB(nullptr),
  Dispatcher(nullptr),
  DispatcherSwitch(nullptr),
  Binary(Binary),
  CurrentCFGForm(CFGForm::UnknownForm),
  createCSAA(createCSAA),
  PCH(PCH) {

  FunctionType *ExitTBTy = FunctionType::get(Type::getVoidTy(Context),
                                             { Type::getInt32Ty(Context) },
                                             false);
  FunctionCallee ExitCallee = TheModule.getOrInsertFunction("exitTB", ExitTBTy);
  ExitTB = cast<Function>(ExitCallee.getCallee());
  FunctionTags::Marker.addTo(ExitTB);

  prepareDispatcher();

  for (auto &Segment : Binary.segments())
    Segment.insertExecutableRanges(std::back_inserter(ExecutableRanges));

  // Configure GlobalValueNumbering
  StringMap<cl::Option *> &Options(cl::getRegisteredOptions());
  getOption<bool>(Options, "enable-load-pre")->setInitialValue(false);
  getOption<unsigned>(Options, "memdep-block-scan-limit")->setInitialValue(100);
  // getOption<bool>(Options, "enable-pre")->setInitialValue(false);
  // getOption<uint32_t>(Options, "max-recurse-depth")->setInitialValue(10);
}

void JumpTargetManager::harvestGlobalData() {
  // Register symbols
  for (auto &P : Binary.labels())
    for (const Label *L : P.second)
      if (L->isSymbol() and L->isCode())
        registerJT(L->address(), JTReason::FunctionSymbol);

  // Register landing pads, if available
  // TODO: should register them in UnusedCodePointers?
  for (MetaAddress LandingPad : Binary.landingPads())
    registerJT(LandingPad, JTReason::GlobalData);

  for (MetaAddress CodePointer : Binary.codePointers())
    registerJT(CodePointer, JTReason::GlobalData);

  for (auto &Segment : Binary.segments()) {
    const Constant *Initializer = Segment.Variable->getInitializer();
    if (isa<ConstantAggregateZero>(Initializer))
      continue;

    auto *Data = cast<ConstantDataArray>(Initializer);
    MetaAddress StartVirtualAddress = Segment.StartVirtualAddress;
    const unsigned char *DataStart = Data->getRawDataValues().bytes_begin();
    const unsigned char *DataEnd = Data->getRawDataValues().bytes_end();

    using endianness = support::endianness;
    if (Binary.architecture().pointerSize() == 64) {
      if (Binary.architecture().isLittleEndian())
        findCodePointers<uint64_t, endianness::little>(StartVirtualAddress,
                                                       DataStart,
                                                       DataEnd);
      else
        findCodePointers<uint64_t, endianness::big>(StartVirtualAddress,
                                                    DataStart,
                                                    DataEnd);
    } else if (Binary.architecture().pointerSize() == 32) {
      if (Binary.architecture().isLittleEndian())
        findCodePointers<uint32_t, endianness::little>(StartVirtualAddress,
                                                       DataStart,
                                                       DataEnd);
      else
        findCodePointers<uint32_t, endianness::big>(StartVirtualAddress,
                                                    DataStart,
                                                    DataEnd);
    }
  }

  revng_log(JTCountLog,
            "JumpTargets found in global data: " << std::dec
                                                 << Unexplored.size());
}

template<typename value_type, unsigned endian>
void JumpTargetManager::findCodePointers(MetaAddress StartVirtualAddress,
                                         const unsigned char *Start,
                                         const unsigned char *End) {
  using support::endianness;
  using support::endian::read;
  for (auto Pos = Start; Pos < End - sizeof(value_type); Pos++) {
    auto Read = read<value_type, static_cast<endianness>(endian), 1>;
    uint64_t RawValue = Read(Pos);
    MetaAddress Value = fromPC(RawValue);
    if (Value.isInvalid())
      continue;

    BasicBlock *Result = registerJT(Value, JTReason::GlobalData);

    if (Result != nullptr)
      UnusedCodePointers.insert(StartVirtualAddress + (Pos - Start));
  }
}

/// Handle a new program counter. We might already have a basic block for that
/// program counter, or we could even have a translation for it. Return one of
/// these, if appropriate.
///
/// \param PC the new program counter.
/// \param ShouldContinue an out parameter indicating whether the returned
///        basic block was just a placeholder or actually contains a
///        translation.
///
/// \return the basic block to use from now on, or null if the program counter
///         is not associated to a basic block.
// TODO: make this return a pair
BasicBlock *JumpTargetManager::newPC(MetaAddress PC, bool &ShouldContinue) {
  revng_assert(PC.isValid());

  // Did we already meet this PC?
  auto JTIt = JumpTargets.find(PC);
  if (JTIt != JumpTargets.end()) {
    // If it was planned to explore it in the future, just to do it now
    for (auto UnexploredIt = Unexplored.begin();
         UnexploredIt != Unexplored.end();
         UnexploredIt++) {

      if (UnexploredIt->first == PC) {
        BasicBlock *Result = UnexploredIt->second;

        // Check if we already have a translation for that
        ShouldContinue = Result->empty();
        if (ShouldContinue) {
          // We don't, OK let's explore it next
          Unexplored.erase(UnexploredIt);
        } else {
          // We do, it will be purged at the next `peek`
          revng_assert(ToPurge.count(Result) != 0);
        }

        return Result;
      }
    }

    // It wasn't planned to visit it, so we've already been there, just jump
    // there
    BasicBlock *BB = JTIt->second.head();
    revng_assert(!BB->empty());
    ShouldContinue = false;
    return BB;
  }

  // Check if we already translated this PC even if it's not associated to a
  // basic block (i.e., we have to split its basic block). This typically
  // happens with variable-length instruction encodings.
  if (OriginalInstructionAddresses.count(PC) != 0) {
    ShouldContinue = false;
    return registerJT(PC, JTReason::AmbigousInstruction);
  }

  // We don't know anything about this PC
  return nullptr;
}

/// Save the PC-Instruction association for future use (jump target)
void JumpTargetManager::registerInstruction(MetaAddress PC,
                                            Instruction *Instruction) {
  revng_assert(PC.isValid());

  // Never save twice a PC
  revng_assert(!OriginalInstructionAddresses.count(PC));
  OriginalInstructionAddresses[PC] = Instruction;
}

// TODO: this is a candidate for BFSVisit
std::pair<MetaAddress, uint64_t>
JumpTargetManager::getPC(Instruction *TheInstruction) const {
  CallInst *NewPCCall = nullptr;
  std::set<BasicBlock *> Visited;
  std::queue<BasicBlock::reverse_iterator> WorkList;
  if (TheInstruction->getIterator() == TheInstruction->getParent()->begin())
    WorkList.push(--TheInstruction->getParent()->rend());
  else
    WorkList.push(++TheInstruction->getReverseIterator());

  while (!WorkList.empty()) {
    auto I = WorkList.front();
    WorkList.pop();
    auto *BB = I->getParent();
    auto End = BB->rend();

    // Go through the instructions looking for calls to newpc
    for (; I != End; I++) {
      if (auto Marker = dyn_cast<CallInst>(&*I)) {
        // TODO: comparing strings is not very elegant
        auto *Callee = Marker->getCalledFunction();
        if (Callee != nullptr && Callee->getName() == "newpc") {

          // We found two distinct newpc leading to the requested instruction
          if (NewPCCall != nullptr)
            return { MetaAddress::invalid(), 0 };

          NewPCCall = Marker;
          break;
        }
      }
    }

    // If we haven't find a newpc call yet, continue exploration backward
    if (NewPCCall == nullptr) {
      // If one of the predecessors is the dispatcher, don't explore any further
      for (BasicBlock *Predecessor : predecessors(BB)) {
        // Assert we didn't reach the almighty dispatcher
        using GCBI = GeneratedCodeBasicInfo;
        revng_assert(not(NewPCCall == nullptr
                         and GCBI::isPartOfRootDispatcher(Predecessor)));
        if (GCBI::isPartOfRootDispatcher(Predecessor))
          continue;
      }

      for (BasicBlock *Predecessor : predecessors(BB)) {
        // Ignore already visited or empty BBs
        if (!Predecessor->empty()
            && Visited.find(Predecessor) == Visited.end()) {
          WorkList.push(Predecessor->rbegin());
          Visited.insert(Predecessor);
        }
      }
    }
  }

  // Couldn't find the current PC
  if (NewPCCall == nullptr)
    return { MetaAddress::invalid(), 0 };

  auto PC = MetaAddress::fromConstant(NewPCCall->getArgOperand(0));
  uint64_t Size = getLimitedValue(NewPCCall->getArgOperand(1));
  revng_assert(Size != 0);
  return { PC, Size };
}

/// \brief Class to iterate over all the BBs associated to a translated PC
class BasicBlockVisitor {
public:
  BasicBlockVisitor(const SwitchInst *Dispatcher) :
    Dispatcher(Dispatcher),
    JumpTargetIndex(0),
    JumpTargetsCount(Dispatcher->getNumSuccessors()),
    DL(Dispatcher->getParent()->getParent()->getParent()->getDataLayout()) {}

  void enqueue(BasicBlock *BB) {
    if (Visited.count(BB))
      return;
    Visited.insert(BB);

    MetaAddress PC = getPC(BB);
    if (PC.isInvalid())
      SamePC.push(BB);
    else
      NewPC.push({ BB, PC });
  }

  std::pair<BasicBlock *, MetaAddress> pop() {
    if (!SamePC.empty()) {
      auto Result = SamePC.front();
      SamePC.pop();
      return { Result, MetaAddress::invalid() };
    } else if (!NewPC.empty()) {
      auto Result = NewPC.front();
      NewPC.pop();
      return Result;
    } else if (JumpTargetIndex < JumpTargetsCount) {
      BasicBlock *BB = Dispatcher->getSuccessor(JumpTargetIndex);
      JumpTargetIndex++;
      return { BB, getPC(BB) };
    } else {
      return { nullptr, MetaAddress::invalid() };
    }
  }

private:
  MetaAddress getPC(BasicBlock *BB) {
    if (!BB->empty()) {
      if (auto *Call = dyn_cast<CallInst>(&*BB->begin())) {
        Function *Callee = Call->getCalledFunction();
        // TODO: comparing with "newpc" string is sad
        if (Callee != nullptr && Callee->getName() == "newpc") {
          return MetaAddress::fromConstant(Call->getArgOperand(0));
        }
      }
    }

    return MetaAddress::invalid();
  }

private:
  const SwitchInst *Dispatcher;
  unsigned JumpTargetIndex;
  unsigned JumpTargetsCount;
  const DataLayout &DL;
  std::set<BasicBlock *> Visited;
  std::queue<BasicBlock *> SamePC;
  std::queue<std::pair<BasicBlock *, MetaAddress>> NewPC;
};

void JumpTargetManager::fixPostHelperPC() {
  for (BasicBlock &BB : *TheFunction) {
    for (Instruction &I : BB) {
      if (auto *Call = getCallToHelper(&I)) {
        using GCBI = GeneratedCodeBasicInfo;
        auto Written = std::move(GCBI::getCSVUsedByHelperCall(Call).Written);
        auto WritesPC = [this](GlobalVariable *CSV) {
          return PCH->affectsPC(CSV);
        };
        auto End = Written.end();
        if (std::find_if(Written.begin(), End, WritesPC) != End) {
          IRBuilder<> Builder(Call->getParent(), ++Call->getIterator());
          PCH->deserializePC(Builder);
        }
      }
    }
  }
}

void JumpTargetManager::translateIndirectJumps() {
  if (ExitTB->use_empty())
    return;

  auto I = ExitTB->use_begin();
  while (I != ExitTB->use_end()) {
    Use &ExitTBUse = *I++;
    if (auto *Call = dyn_cast<CallInst>(ExitTBUse.getUser())) {
      if (Call->getCalledFunction() == ExitTB) {

        // Look for the last write to the PC
        BasicBlock *CallBB = Call->getParent();
        auto [Result, NextPC] = PCH->getUniqueJumpTarget(CallBB);
        revng_check(Result != NextJumpTarget::Unique
                    and "Direct jumps should not be handled here");

        if (getLimitedValue(Call->getArgOperand(0)) == 0) {
          exitTBCleanup(Call);
          BranchInst::Create(AnyPC, Call);
        }

        Call->eraseFromParent();
      }
    }
  }

  revng_assert(ExitTB->use_empty());
  ExitTB->eraseFromParent();
  ExitTB = nullptr;
}

JumpTargetManager::BlockWithAddress JumpTargetManager::peek() {
  // If we just harvested new branches, keep exploring
  do {
    harvest();
  } while (Unexplored.empty() and NewBranches != 0);

  // Purge all the partial translations we know might be wrong
  for (BasicBlock *BB : ToPurge)
    purgeTranslation(BB);
  ToPurge.clear();

  if (Unexplored.empty()) {
    revng_log(JTCountLog, "We're done looking for jump targets");
    return NoMoreTargets;
  } else {
    BlockWithAddress Result = Unexplored.back();
    Unexplored.pop_back();
    return Result;
  }
}

BasicBlock *JumpTargetManager::getBlockAt(MetaAddress PC) {
  revng_assert(PC.isValid());

  auto TargetIt = JumpTargets.find(PC);
  revng_assert(TargetIt != JumpTargets.end());
  return TargetIt->second.head();
}

/// \brief Check if among \p BB's predecessors there's \p Target
inline bool hasRootDispatcherPredecessor(llvm::BasicBlock *BB) {
  using GCBI = GeneratedCodeBasicInfo;
  for (llvm::BasicBlock *Predecessor : predecessors(BB))
    if (GCBI::isPartOfRootDispatcher(Predecessor))
      return true;
  return false;
}

void JumpTargetManager::purgeTranslation(BasicBlock *Start) {
  OnceQueue<BasicBlock *> Queue;
  Queue.insert(Start);

  // Collect all the descendants, except if we meet a jump target
  while (!Queue.empty()) {
    BasicBlock *BB = Queue.pop();
    Instruction *Terminator = BB->getTerminator();
    for (BasicBlock *Successor : successors(Terminator)) {
      if (isTranslatedBB(Successor) and not isJumpTarget(Successor)
          and not hasRootDispatcherPredecessor(Successor)) {
        Queue.insert(Successor);
      }
    }
  }

  // Erase all the visited basic blocks
  std::set<BasicBlock *> Visited = Queue.visited();

  // Build a subgraph, so that we can visit it in post order, and purge the
  // content of each basic block
  SubGraph<BasicBlock *> TranslatedBBs(Start, Visited);
  for (auto *Node : post_order(TranslatedBBs)) {
    BasicBlock *BB = Node->get();
    while (!BB->empty()) {
      Instruction *I = &*(--BB->end());

      if (CallInst *Call = getCallTo(I, "newpc")) {
        auto *Address = Call->getArgOperand(0);
        OriginalInstructionAddresses.erase(MetaAddress::fromConstant(Address));
      }
      eraseInstruction(I);
    }
  }

  // Remove Start, since we want to keep it (even if empty)
  Visited.erase(Start);

  for (BasicBlock *BB : Visited) {
    // We might have some predecessorless basic blocks jumping to us, purge them
    // TODO: why this?
    while (pred_begin(BB) != pred_end(BB)) {
      BasicBlock *Predecessor = *pred_begin(BB);
      revng_assert(pred_empty(Predecessor));
      Predecessor->eraseFromParent();
    }

    revng_assert(BB->use_empty());
    BB->eraseFromParent();
  }
}

// TODO: register Reason
BasicBlock *
JumpTargetManager::registerJT(MetaAddress PC, JTReason::Values Reason) {
  revng_check(PC.isValid());

  if (not isPC(PC))
    return nullptr;

  revng_log(RegisterJTLog,
            "Registering bb." << nameForAddress(PC) << " for "
                              << JTReason::getName(Reason));

  // Do we already have a BasicBlock for this PC?
  BlockMap::iterator TargetIt = JumpTargets.find(PC);
  if (TargetIt != JumpTargets.end()) {
    // Case 1: there's already a BasicBlock for that address, return it
    BasicBlock *BB = TargetIt->second.head();
    TargetIt->second.setReason(Reason);
    return BB;
  }

  // Did we already meet this PC (i.e. do we know what's the associated
  // instruction)?
  BasicBlock *NewBlock = nullptr;
  InstructionMap::iterator InstrIt = OriginalInstructionAddresses.find(PC);
  if (InstrIt != OriginalInstructionAddresses.end()) {
    // Case 2: the address has already been met, but needs to be promoted to
    //         BasicBlock level.
    Instruction *I = InstrIt->second;
    BasicBlock *ContainingBlock = I->getParent();
    if (isFirst(I)) {
      NewBlock = ContainingBlock;
    } else {
      revng_assert(I != nullptr && I->getIterator() != ContainingBlock->end());
      NewBlock = ContainingBlock->splitBasicBlock(I);
    }

    // Register the basic block and all of its descendants to be purged so that
    // we can retranslate this PC
    // TODO: this might create a problem if QEMU generates control flow that
    //       crosses an instruction boundary
    ToPurge.insert(NewBlock);

  } else {
    // Case 3: the address has never been met, create a temporary one, register
    // it for future exploration and return it
    NewBlock = BasicBlock::Create(Context, "", TheFunction);
  }

  Unexplored.push_back(BlockWithAddress(PC, NewBlock));

  std::stringstream Name;
  Name << "bb." << nameForAddress(PC);
  NewBlock->setName(Name.str());

  // Create a case for the address associated to the new block, if the
  // dispatcher has alredy been emitted
  if (DispatcherSwitch != nullptr) {
    PCH->addCaseToDispatcher(DispatcherSwitch,
                             { PC, NewBlock },
                             BlockType::RootDispatcherHelperBlock);
  }

  // Associate the PC with the chosen basic block
  JumpTargets[PC] = JumpTarget(NewBlock, Reason);

  // PC was not a jump target, record it as new
  AVIPCWhiteList.insert(PC);

  return NewBlock;
}

void JumpTargetManager::registerReadRange(MetaAddress Address, uint64_t Size) {
  using interval = boost::icl::interval<MetaAddress, compareAddress>;
  ReadIntervalSet += interval::right_open(Address, Address + Size);
}

void JumpTargetManager::prepareDispatcher() {
  IRBuilder<> Builder(Context);
  QuickMetadata QMD(Context);

  // Create the first block of the dispatcher
  BasicBlock *Entry = BasicBlock::Create(Context,
                                         "dispatcher.entry",
                                         TheFunction);

  // The default case of the switch statement it's an unhandled cases
  DispatcherFail = BasicBlock::Create(Context,
                                      "dispatcher.default",
                                      TheFunction);
  Builder.SetInsertPoint(DispatcherFail);

  auto *UnknownPCTy = FunctionType::get(Type::getVoidTy(Context),
                                        { MetaAddress::getStruct(&TheModule) },
                                        false);
  FunctionCallee UnknownPC = TheModule.getOrInsertFunction("unknownPC",
                                                           UnknownPCTy);
  {
    auto *UnknownPCFunction = cast<Function>(skipCasts(UnknownPC.getCallee()));
    FunctionTags::Exceptional.addTo(UnknownPCFunction);
  }

  Builder.CreateCall(UnknownPC, PCH->loadPC(Builder));
  auto *FailUnreachable = Builder.CreateUnreachable();
  setBlockType(FailUnreachable, BlockType::DispatcherFailureBlock);

  Dispatcher = Entry;

  // Create basic blocks to handle jumps to any PC and to a PC we didn't expect
  AnyPC = BasicBlock::Create(Context, "anypc", TheFunction);
  UnexpectedPC = BasicBlock::Create(Context, "unexpectedpc", TheFunction);
}

static void purge(BasicBlock *BB) {
  // Allow up to a single instruction in the basic block
  if (!BB->empty())
    BB->begin()->eraseFromParent();
  revng_assert(BB->empty());
}

std::set<BasicBlock *> JumpTargetManager::computeUnreachable() const {
  ReversePostOrderTraversal<BasicBlock *> RPOT(&TheFunction->getEntryBlock());
  std::set<BasicBlock *> Reachable;
  for (BasicBlock *BB : RPOT)
    Reachable.insert(BB);

  // TODO: why is isTranslatedBB(&BB) necessary?
  std::set<BasicBlock *> Unreachable;
  for (BasicBlock &BB : *TheFunction)
    if (Reachable.count(&BB) == 0 and isTranslatedBB(&BB))
      Unreachable.insert(&BB);

  return Unreachable;
}

void JumpTargetManager::setCFGForm(CFGForm::Values NewForm,
                                   MetaAddressSet *JumpTargetsWhitelist) {
  revng_assert(CurrentCFGForm != NewForm);
  revng_assert(NewForm != CFGForm::UnknownForm);

  std::set<BasicBlock *> Unreachable;
  static bool First = true;
  if (not First and VerifyLog.isEnabled()) {
    assertNoUnreachable();
  }
  First = false;

  CFGForm::Values OldForm = CurrentCFGForm;
  CurrentCFGForm = NewForm;

  //
  // Recreate AnyPC and UnexpectedPC
  //
  switch (NewForm) {
  case CFGForm::SemanticPreserving:
    purge(AnyPC);
    BranchInst::Create(dispatcher(), AnyPC);
    // TODO: Here we should have an hard fail, since it's the situation in
    //       which we expected to know where execution could go but we made a
    //       mistake.
    purge(UnexpectedPC);
    BranchInst::Create(dispatcher(), UnexpectedPC);
    break;

  case CFGForm::RecoveredOnly:
  case CFGForm::NoFunctionCalls:
    purge(AnyPC);
    new UnreachableInst(Context, AnyPC);
    purge(UnexpectedPC);
    new UnreachableInst(Context, UnexpectedPC);
    break;

  default:
    revng_abort("Not implemented yet");
  }

  setBlockType(AnyPC->getTerminator(), BlockType::AnyPCBlock);
  setBlockType(UnexpectedPC->getTerminator(), BlockType::UnexpectedPCBlock);

  // Adjust successors of jump instructions tagged as function calls:
  //
  // * NoFunctionsCallsCFG: the successor is the fallthrough
  // * otherwise: the successor is the callee
  if (NewForm == CFGForm::NoFunctionCalls
      || OldForm == CFGForm::NoFunctionCalls) {
    if (auto *FunctionCall = TheModule.getFunction("function_call")) {
      for (User *U : FunctionCall->users()) {
        auto *Call = cast<CallInst>(U);

        // Ignore indirect calls
        // TODO: why this is needed is unclear
        if (isa<ConstantPointerNull>(Call->getArgOperand(0)))
          continue;

        Instruction *Terminator = nextNonMarker(Call);
        revng_assert(Terminator->getNumSuccessors() == 1);

        // Get the correct argument, the first is the callee, the second the
        // return basic block
        int OperandIndex = NewForm == CFGForm::NoFunctionCalls ? 1 : 0;
        Value *Op = Call->getArgOperand(OperandIndex);
        BasicBlock *NewSuccessor = cast<BlockAddress>(Op)->getBasicBlock();
        Terminator->setSuccessor(0, NewSuccessor);
      }
    }
  }

  rebuildDispatcher(JumpTargetsWhitelist);

  if (VerifyLog.isEnabled()) {
    assertNoUnreachable();
  }
}

void JumpTargetManager::rebuildDispatcher(MetaAddressSet *Whitelist) {

  if (DispatcherSwitch != nullptr) {
    revng_assert(DispatcherSwitch->getParent() == Dispatcher);

    // Purge the old dispatcher
    PCH->destroyDispatcher(DispatcherSwitch);
  }

  ProgramCounterHandler::DispatcherTargets Targets;

  // Add all the (whitelisted) jump targets if we're using the
  // SemanticPreserving, or only those with no predecessors.
  bool IsWhitelistActive = (Whitelist != nullptr);
  for (auto &[PC, JumpTarget] : JumpTargets) {
    BasicBlock *BB = JumpTarget.head();
    bool IsWhitelisted = (not IsWhitelistActive or Whitelist->count(PC) != 0);
    if ((CurrentCFGForm == CFGForm::SemanticPreserving
         or not hasPredecessors(BB))
        and IsWhitelisted) {
      Targets.emplace_back(PC, BB);
    }
  }

  constexpr auto RDHB = BlockType::RootDispatcherHelperBlock;
  const auto &DispatcherInfo = PCH->buildDispatcher(Targets,
                                                    Dispatcher,
                                                    DispatcherFail,
                                                    RDHB);
  DispatcherSwitch = DispatcherInfo.Switch;

  // The switch is the terminator of the dispatcher basic block
  setBlockType(DispatcherSwitch, BlockType::RootDispatcherBlock);

  //
  // Make sure every generated basic block is reachable
  //
  if (CurrentCFGForm != CFGForm::SemanticPreserving) {
    // Compute the set of reachable jump targets
    OnceQueue<BasicBlock *> WorkList;
    for (BasicBlock *Successor : successors(DispatcherSwitch))
      WorkList.insert(Successor);

    while (not WorkList.empty()) {
      BasicBlock *BB = WorkList.pop();
      for (BasicBlock *Successor : make_range(succ_begin(BB), succ_end(BB)))
        WorkList.insert(Successor);
    }

    std::set<BasicBlock *> Reachable = WorkList.visited();

    // Identify all the unreachable jump targets
    for (const auto &[PC, JT] : JumpTargets) {
      BasicBlock *BB = JT.head();
      bool IsWhitelisted = (not IsWhitelistActive or Whitelist->count(PC) != 0);

      // Add to the switch all the unreachable jump targets whose reason is not
      // just direct jump
      if (Reachable.count(BB) == 0 and IsWhitelisted
          and not JT.isOnlyReason(JTReason::DirectJump)) {
        PCH->addCaseToDispatcher(DispatcherSwitch,
                                 { PC, BB },
                                 BlockType::RootDispatcherHelperBlock);
      }
    }
  }
}

bool JumpTargetManager::hasPredecessors(BasicBlock *BB) const {
  for (BasicBlock *Pred : predecessors(BB))
    if (isTranslatedBB(Pred))
      return true;
  return false;
}

/// \brief Simple pass to drop `range` metadata, which is sometimes detrimental
class DropRangeMetadataPass : public PassInfoMixin<DropRangeMetadataPass> {

public:
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &) {
    for (BasicBlock &BB : F)
      for (Instruction &I : BB)
        I.setMetadata("range", nullptr);
    return PreservedAnalyses::none();
  }
};

/// \brief Drop all the call to marker functions
class DropMarkerCalls : public PassInfoMixin<DropMarkerCalls> {
private:
  SmallVector<StringRef, 4> ToPreserve;

public:
  DropMarkerCalls(SmallVector<StringRef, 4> ToPreserve) :
    ToPreserve(ToPreserve) {}

public:
  PreservedAnalyses run(Function &F, FunctionAnalysisManager &) {
    Module *M = F.getParent();
    std::vector<CallInst *> ToErase;

    for (Function &Marker : FunctionTags::Marker.functions(M)) {
      //
      // Check if we should preserve this marker
      //
      auto It = std::find(ToPreserve.begin(),
                          ToPreserve.end(),
                          Marker.getName());
      auto End = std::end(ToPreserve);
      if (It != End) {
        Marker.setDoesNotReturn();
        continue;
      }

      //
      // Register all the calls to be erased
      //
      for (User *U : Marker.users())
        if (auto *Call = dyn_cast<CallInst>(U))
          if (Call->getParent()->getParent() == &F)
            ToErase.push_back(Call);
    }

    //
    // Actually drop the calls
    //
    for (CallInst *Call : ToErase)
      Call->eraseFromParent();

    return PreservedAnalyses::none();
  }
};

void JumpTargetManager::aliasAnalysis() {
  unsigned AliasScopeMDKindID = TheModule.getMDKindID("alias.scope");
  unsigned NoAliasMDKindID = TheModule.getMDKindID("noalias");

  LLVMContext &Context = TheModule.getContext();
  QuickMetadata QMD(Context);
  MDBuilder MDB(Context);
  MDNode *CSVDomain = MDB.createAliasScopeDomain("CSVAliasDomain");

  struct CSVAliasInfo {
    MDNode *AliasScope;
    MDNode *AliasSet;
    MDNode *NoAliasSet;
  };
  std::map<const GlobalVariable *, CSVAliasInfo> CSVAliasInfoMap;

  std::set<GlobalVariable *> CSVs;
  NamedMDNode *NamedMD = TheModule.getOrInsertNamedMetadata("revng.csv");
  auto *Tuple = cast<MDTuple>(NamedMD->getOperand(0));
  for (const MDOperand &Operand : Tuple->operands()) {
    auto *CSV = cast<GlobalVariable>(QMD.extract<Constant *>(Operand.get()));
    CSVs.insert(CSV);
  }

  for (GlobalVariable *PCCSV : PCH->pcCSVs())
    CSVs.insert(PCCSV);

  // Build alias scopes
  std::vector<Metadata *> AllCSVScopes;
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];

    std::string Name = CSV->getName().str();
    MDNode *CSVScope = MDB.createAliasScope(Name, CSVDomain);
    AliasInfo.AliasScope = CSVScope;
    AllCSVScopes.push_back(CSVScope);
    MDNode *CSVAliasSet = MDNode::get(Context,
                                      ArrayRef<Metadata *>({ CSVScope }));
    AliasInfo.AliasSet = CSVAliasSet;
  }
  MDNode *MemoryAliasSet = MDNode::get(Context, AllCSVScopes);

  // Build noalias sets
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];
    std::vector<Metadata *> OtherCSVScopes;
    for (const auto &Q : CSVAliasInfoMap)
      if (Q.first != CSV)
        OtherCSVScopes.push_back(Q.second.AliasScope);

    MDNode *CSVNoAliasSet = MDNode::get(Context, OtherCSVScopes);
    AliasInfo.NoAliasSet = CSVNoAliasSet;
  }

  // Decorate the IR with alias information
  for (Function &F : TheModule) {
    for (BasicBlock &BB : F) {
      for (Instruction &I : BB) {
        Value *Ptr = nullptr;

        if (auto *L = dyn_cast<LoadInst>(&I))
          Ptr = L->getPointerOperand();
        else if (auto *S = dyn_cast<StoreInst>(&I))
          Ptr = S->getPointerOperand();
        else
          continue;

        // Check if the pointer is a CSV
        if (auto *GV = dyn_cast<GlobalVariable>(Ptr)) {
          auto It = CSVAliasInfoMap.find(GV);
          if (It != CSVAliasInfoMap.end()) {
            // Set alias.scope and noalias metadata
            I.setMetadata(AliasScopeMDKindID, It->second.AliasSet);
            I.setMetadata(NoAliasMDKindID, It->second.NoAliasSet);
            continue;
          }
        }

        // It's not a CSV memory access, set noalias info
        I.setMetadata(NoAliasMDKindID, MemoryAliasSet);
      }
    }
  }
}

namespace TrackedInstructionType {

enum Values { Invalid, WrittenInPC, StoredInMemory, StoreTarget, LoadTarget };

inline const char *getName(Values V) {
  switch (V) {
  case Invalid:
    return "Invalid";
  case WrittenInPC:
    return "WrittenInPC";
  case StoredInMemory:
    return "StoredInMemory";
  case StoreTarget:
    return "StoreTarget";
  case LoadTarget:
    return "LoadTarget";
  default:
    revng_abort();
  }
}

inline Values fromName(llvm::StringRef Name) {
  if (Name == "Invalid")
    return Invalid;
  else if (Name == "WrittenInPC")
    return WrittenInPC;
  else if (Name == "StoredInMemory")
    return StoredInMemory;
  else if (Name == "StoreTarget")
    return StoreTarget;
  else if (Name == "LoadTarget")
    return LoadTarget;
  else
    revng_abort();
}

} // namespace TrackedInstructionType

class AnalysisRegistry {
public:
  using TrackedValueType = TrackedInstructionType::Values;

  struct TrackedValue {
    MetaAddress Address;
    TrackedValueType Type;
    Instruction *I;
  };

private:
  std::vector<TrackedValue> TrackedValues;
  QuickMetadata QMD;
  llvm::Function *AVIMarker;
  IRBuilder<> Builder;

public:
  AnalysisRegistry(Module *M) : QMD(getContext(M)), Builder(getContext(M)) {
    AVIMarker = AdvancedValueInfoPass::createMarker(M);
  }

  llvm::Function *aviMarker() const { return AVIMarker; }

  void registerValue(MetaAddress Address,
                     Value *OriginalValue,
                     Value *ValueToTrack,
                     TrackedValueType Type) {
    revng_assert(Address.isValid());

    Instruction *InstructionToTrack = dyn_cast<Instruction>(ValueToTrack);
    if (InstructionToTrack == nullptr)
      return;

    revng_assert(InstructionToTrack != nullptr);

    // Create the marker call and attach as second argument a unique
    // identifier. This is necessary since the instruction itself could be
    // deleted, duplicated and what not. Later on, we will use TrackedValues
    // to now the values that have been identified to which value in the
    // original function did they belong to
    uint32_t AVIID = TrackedValues.size();
    Builder.SetInsertPoint(InstructionToTrack->getNextNode());
    Builder.CreateCall(AVIMarker,
                       { InstructionToTrack, Builder.getInt32(AVIID) });
    TrackedValue NewTV{ Address,
                        Type,
                        cast_or_null<Instruction>(OriginalValue) };
    TrackedValues.push_back(NewTV);
  }

  const TrackedValue &rootInstructionById(uint32_t ID) const {
    return TrackedValues.at(ID);
  }
};

JumpTargetManager::MetaAddressSet JumpTargetManager::inflateAVIWhitelist() {
  MetaAddressSet Result;

  // We start from all the new basic blocks (i.e., those in
  // AVIPCWhiteList) and proceed backward in the CFG in order to
  // whitelist all the jump targets we meet. We stop when we meet the dispatcher
  // or a function call.

  // Prepare the backward visit
  df_iterator_default_set<BasicBlock *> VisitSet;

  // Stop at the dispatcher
  VisitSet.insert(DispatcherSwitch->getParent());

  // TODO: OriginalInstructionAddresses is not reliable, we should drop it
  for (User *NewPCUser : TheModule.getFunction("newpc")->users()) {
    auto *I = cast<Instruction>(NewPCUser);
    auto WhitelistedMA = getPCFromNewPCCall(I);
    if (WhitelistedMA.isValid()) {
      if (AVIPCWhiteList.count(WhitelistedMA) != 0) {
        BasicBlock *BB = I->getParent();
        auto VisitRange = inverse_depth_first_ext(BB, VisitSet);
        for (const BasicBlock *Reachable : VisitRange) {
          auto MA = getPCFromNewPCCall(&*Reachable->begin());
          if (MA.isValid() and isJumpTarget(MA)) {
            Result.insert(MA);
          }
        }
      }
    }
  }

  return Result;
}

void JumpTargetManager::harvestWithAVI() {
  Module *M = TheFunction->getParent();

  //
  // Update CPUStateAccessAnalysisPass
  //
  legacy::PassManager PM;
  PM.add(createCSAA());
  PM.add(new FunctionCallIdentification);
  PM.run(TheModule);

  //
  // Collect all the non-PC affectign CSVs
  //
  std::set<GlobalVariable *> NonPCCSVs;
  QuickMetadata QMD(Context);
  NamedMDNode *NamedMD = TheModule.getOrInsertNamedMetadata("revng.csv");
  auto *Tuple = cast<MDTuple>(NamedMD->getOperand(0));
  for (const MDOperand &Operand : Tuple->operands()) {
    auto *CSV = cast<GlobalVariable>(QMD.extract<Constant *>(Operand.get()));
    if (not PCH->affectsPC(CSV))
      NonPCCSVs.insert(CSV);
  }

  //
  // Clone the root function
  //
  Function *OptimizedFunction = nullptr;
  ValueToValueMapTy OldToNew;
  {
    // Break all the call edges. We want to ignore those for CFG recovery
    // purposes.
    std::map<Use *, BasicBlock *> Undo;
    auto *FunctionCall = TheModule.getFunction("function_call");
    revng_assert(FunctionCall != nullptr);
    for (CallBase *Call : callers(FunctionCall)) {
      auto *T = Call->getParent()->getTerminator();
      if (auto *Branch = dyn_cast<BranchInst>(T)) {
        revng_assert(Branch->isUnconditional());
        BasicBlock *Target = Branch->getSuccessor(0);
        Use *U = &Branch->getOperandUse(0);

        // We're after a function call: pretend we're jumping to the
        // dispatcher
        U->set(Dispatcher);

        // Record Use for later undoing
        Undo[U] = Target;
      }
    }

    // Compute AVIJumpTargetWhitelist
    auto AVIJumpTargetWhitelist = inflateAVIWhitelist();

    // Prune the dispatcher
    setCFGForm(CFGForm::RecoveredOnly, &AVIJumpTargetWhitelist);

    // Detach all the unreachable basic blocks, so they don't get copied
    std::set<BasicBlock *> UnreachableBBs = computeUnreachable();
    for (BasicBlock *UnreachableBB : UnreachableBBs)
      UnreachableBB->removeFromParent();

    // Clone the function
    OptimizedFunction = CloneFunction(TheFunction, OldToNew);

    // Restore callees after function_call
    for (auto [U, BB] : Undo)
      U->set(BB);

    // Record the size of OptimizedFunction
    size_t BlocksCount = OptimizedFunction->getBasicBlockList().size();
    BlocksAnalyzedByAVI.push(BlocksCount);

    // Reattach the unreachable basic blocks to the original root function
    for (BasicBlock *UnreachableBB : UnreachableBBs)
      UnreachableBB->insertInto(TheFunction);

    // Restore the dispatcher in the original function
    setCFGForm(CFGForm::SemanticPreserving);
    revng_assert(computeUnreachable().size() == 0);

    // Clear the whitelist
    AVIPCWhiteList.clear();
  }

  //
  // Register for analysis the value written in the PC before each exit_tb call
  //
  AnalysisRegistry AR(M);
  IRBuilder<> Builder(Context);
  for (User *U : ExitTB->users()) {
    if (auto *Call = dyn_cast<CallInst>(U)) {
      BasicBlock *BB = Call->getParent();
      if (BB->getParent() == TheFunction) {
        auto It = OldToNew.find(Call);
        if (It == OldToNew.end())
          continue;
        Builder.SetInsertPoint(cast<CallInst>(&*It->second));
        Instruction *ComposedIntegerPC = PCH->composeIntegerPC(Builder);
        AR.registerValue(getPC(Call).first,
                         Call,
                         ComposedIntegerPC,
                         TrackedInstructionType::WrittenInPC);
      }
    }
  }

  //
  // Register load/store addresses and PC-sized stored value
  //
  for (BasicBlock &BB : *OptimizedFunction) {
    for (Instruction &I : BB) {
      namespace TIT = TrackedInstructionType;

      auto AddressType = TIT::Invalid;
      Value *Pointer = nullptr;
      Instruction *StoredInstruction = nullptr;
      if (auto *Load = dyn_cast<LoadInst>(&I)) {
        // It's a load: record the load address
        Pointer = Load->getPointerOperand();
        AddressType = TIT::LoadTarget;
      } else if (auto *Store = dyn_cast<StoreInst>(&I)) {
        // It's a store: record the store address and the stored value
        Pointer = Store->getPointerOperand();
        StoredInstruction = dyn_cast<Instruction>(Store->getValueOperand());
        AddressType = TIT::StoreTarget;
      }

      // Exclude memory accesses targeting CSVs
      if (Pointer != nullptr and isMemory(Pointer)) {
        MetaAddress Address = getPC(&I).first;
        // Register the load/store address
        if (Instruction *PointerI = dyn_cast_or_null<Instruction>(Pointer))
          AR.registerValue(Address, nullptr, PointerI, AddressType);

        // Register the stored value, if pc-sized
        Type *PointeeType = Pointer->getType()->getPointerElementType();
        if (StoredInstruction != nullptr and PCH->isPCSizedType(PointeeType))
          AR.registerValue(Address,
                           nullptr,
                           StoredInstruction,
                           TIT::StoredInMemory);
      }
    }
  }

  //
  // Create and initialized an alloca per CSV (except for the PC-affecting ones)
  //
  BasicBlock *EntryBB = &OptimizedFunction->getEntryBlock();
  IRBuilder<> AllocaBuilder(&*EntryBB->begin());
  IRBuilder<> InitializeBuilder(EntryBB->getTerminator());
  std::map<GlobalVariable *, AllocaInst *> CSVMap;

  for (GlobalVariable *CSV : NonPCCSVs) {
    Type *CSVType = CSV->getType()->getPointerElementType();
    auto *Alloca = AllocaBuilder.CreateAlloca(CSVType, nullptr, CSV->getName());
    CSVMap[CSV] = Alloca;

    // Replace all uses of the CSV within OptimizedFunction with the alloca
    replaceAllUsesInFunctionWith(OptimizedFunction, CSV, Alloca);

    // Initialize the alloca
    InitializeBuilder.CreateStore(InitializeBuilder.CreateLoad(CSV), Alloca);
  }

  //
  // Helper to intrinsic promotion
  //
  using MapperFunction = std::function<Instruction *(CallInst *)>;
  std::pair<std::vector<StringRef>, MapperFunction> Mapping[] = {
    { { "helper_clz", "helper_clz32", "helper_clz64", "helper_dclz" },
      [&Builder](CallInst *Call) {
        return Builder.CreateBinaryIntrinsic(Intrinsic::ctlz,
                                             Call->getArgOperand(0),
                                             Builder.getFalse());
      } }
  };

  for (auto &[HelperNames, Mapper] : Mapping) {
    for (StringRef HelperName : HelperNames) {
      if (Function *Original = M->getFunction(HelperName)) {

        SmallVector<std::pair<Instruction *, Instruction *>, 16> Replacements;
        for (User *U : Original->users()) {
          if (auto *Call = dyn_cast<CallInst>(U)) {
            if (Call->getParent()->getParent() == OptimizedFunction) {
              Builder.SetInsertPoint(Call);
              Instruction *NewI = Mapper(Call);
              NewI->copyMetadata(*Call);
              Replacements.emplace_back(Call, NewI);
            }
          }
        }

        // Apply replacements
        for (auto &P : Replacements) {
          P.first->replaceAllUsesWith(P.second);
          P.first->eraseFromParent();
        }
      }
    }
  }

  SmallVector<CallInst *, 16> ToErase;
  for (User *U : M->getFunction("newpc")->users()) {
    Instruction *I = dyn_cast<Instruction>(U);
    if (I == nullptr or I->getParent()->getParent() != OptimizedFunction)
      continue;

    auto *Call = getCallTo(I, "newpc");
    if (Call == nullptr)
      continue;

    PCH->expandNewPC(Call);
    ToErase.push_back(Call);
  }

  for (CallInst *Call : ToErase)
    Call->eraseFromParent();

  //
  // Update alias analysis
  //
  aliasAnalysis();

  //
  // Optimize the hell out of it and collect the possible values of indirect
  // branches
  //

  StringRef SyscallHelperName = Binary.architecture().syscallHelper();
  Function *SyscallHelper = M->getFunction(SyscallHelperName);
  StringRef SyscallIDCSVName = Binary.architecture().syscallNumberRegister();
  GlobalVariable *SyscallIDCSV = M->getGlobalVariable(SyscallIDCSVName);

  SummaryCallsBuilder SCB(CSVMap);

  // Remove PC initialization from entry block
  {
    BasicBlock &Entry = OptimizedFunction->getEntryBlock();
    std::vector<Instruction *> ToDelete;
    for (Instruction &I : Entry)
      if (auto *Store = dyn_cast<StoreInst>(&I))
        if (isa<Constant>(Store->getValueOperand()) and PCH->affectsPC(Store))
          ToDelete.push_back(&I);

    for (Instruction *I : ToDelete)
      I->eraseFromParent();
  }

  {
    // Note: it is important to let the pass manager go out of scope ASAP:
    //       LazyValueInfo registers a lot of callbacks to get notified when a
    //       Value is destroyed, slowing down OptimizedFunction->eraseFromParent
    //       enormously.
    FunctionPassManager FPM;
    FPM.addPass(DropMarkerCalls({ "exitTB" }));
    FPM.addPass(DropHelperCallsPass(SyscallHelper, SyscallIDCSV, SCB));
    FPM.addPass(ShrinkInstructionOperandsPass());
    FPM.addPass(PromotePass());
    FPM.addPass(InstCombinePass(true));
    FPM.addPass(TypeShrinking::TypeShrinkingPass());
    FPM.addPass(JumpThreadingPass());
    FPM.addPass(UnreachableBlockElimPass());
    FPM.addPass(InstCombinePass(true));
    FPM.addPass(EarlyCSEPass(true));
    FPM.addPass(DropRangeMetadataPass());
    FPM.addPass(AdvancedValueInfoPass(this));

    FunctionAnalysisManager FAM;
    FAM.registerPass([]() { return TypeShrinking::BitLivenessPass(); });
    FAM.registerPass([] {
      AAManager AA;
      AA.registerFunctionAnalysis<BasicAA>();
      AA.registerFunctionAnalysis<ScopedNoAliasAA>();

      return AA;
    });

    ModuleAnalysisManager MAM;
    auto MAMFunactionProxyFactory = [&MAM] {
      return ModuleAnalysisManagerFunctionProxy(MAM);
    };
    FAM.registerPass(MAMFunactionProxyFactory);

    PassBuilder PB;
    PB.registerFunctionAnalyses(FAM);
    PB.registerModuleAnalyses(MAM);

    FPM.run(*OptimizedFunction, FAM);
  }

  if (VerifyLog.isEnabled())
    revng_check(not verifyModule(*OptimizedFunction->getParent(), &dbgs()));

  //
  // Collect the results
  //

  // Iterate over all the AVI markers
  Function *AVIMarker = AR.aviMarker();
  StructType *MetaAddressStruct = MetaAddress::getStruct(M);
  for (User *U : AVIMarker->users()) {
    auto *Call = dyn_cast<CallInst>(U);
    if (Call == nullptr or skipCasts(Call->getCalledOperand()) != AVIMarker)
      continue;

    // Get the ID from the marker, and then the original instruction and marker
    // type
    uint32_t AVIID = getLimitedValue(cast<ConstantInt>(Call->getArgOperand(1)));
    auto TV = AR.rootInstructionById(AVIID);
    auto TIT = TV.Type;

    // Did AVI produce any info?
    auto *T = dyn_cast_or_null<MDTuple>(Call->getMetadata("revng.avi"));
    if (T == nullptr)
      continue;

    // Is this a direct write to PC?
    bool IsComposedIntegerPC = (TIT == TrackedInstructionType::WrittenInPC);

    // We want to register the results only if *all* of them are good
    bool AllValid = true;
    bool AllPCs = true;

    SmallVector<MetaAddress, 16> Targets;

    // Iterate over all the generated values
    for (const MDOperand &Operand : cast<MDTuple>(T)->operands()) {
      // Extract the value
      auto *Value = QMD.extract<ConstantInt *>(Operand.get());

      // Deserialize value into a MetaAddress, depending on the tracked
      // instruction type
      auto MA = (IsComposedIntegerPC ?
                   MetaAddress::decomposeIntegerPC(Value) :
                   MetaAddress::fromPC(TV.Address, getLimitedValue(Value)));

      if (MA.isInvalid()) {
        AllValid = false;
        break;
      }

      if (not isPC(MA))
        AllPCs = false;

      Targets.push_back(MA);
    }

    // Proceed only if all the results are valid
    if (not AllValid)
      continue;

    // If it's supposed to be a PC, all of them have to be a PC
    bool ShouldBePC = (TIT == TrackedInstructionType::WrittenInPC
                       or TIT == TrackedInstructionType::StoredInMemory);
    if (ShouldBePC and not AllPCs)
      continue;

    // Register the resulting addresses
    SmallVector<Metadata *, 16> TargetsMD;
    for (const MetaAddress &MA : Targets) {
      switch (TIT) {
      case TrackedInstructionType::WrittenInPC:
        registerJT(MA, JTReason::PCStore);
        break;

      case TrackedInstructionType::StoredInMemory:
        registerJT(MA, JTReason::MemoryStore);
        break;

      case TrackedInstructionType::StoreTarget:
      case TrackedInstructionType::LoadTarget:
        markJT(MA, JTReason::LoadAddress);
        break;

      case TrackedInstructionType::Invalid:
        revng_abort();
      }

      // Turn the MetaAddress in a Constant(AsMetadata)
      TargetsMD.push_back(QMD.get(MA.toConstant(MetaAddressStruct)));
    }

    // Save the results in root too for pinAVIResults to use
    if (TIT == TrackedInstructionType::WrittenInPC) {
      TV.I->setMetadata("revng.targets", QMD.tuple(TargetsMD));
    }

    auto OperandsCount = Targets.size();
    if (OperandsCount != 0 and NewEdgesLog.isEnabled()) {
      revng_log(NewEdgesLog,
                OperandsCount << " targets from " << getName(Call));
    }
  }

  //
  // Drop the optimized function
  //
  OptimizedFunction->eraseFromParent();

  // Drop temporary functions
  SCB.cleanup();
}

// Harvesting proceeds trying to avoid to run expensive analyses if not strictly
// necessary. To do this we keep in mind two aspects: do we have new basic
// blocks to visit? If so, we avoid any further anyalysis and give back control
// to the translator. If not, we proceed with other analyses until we either
// find a new basic block to translate. If we can't find a new block to
// translate we proceed as long as we are able to create new edges on the CFG
// (not considering the dispatcher).
void JumpTargetManager::harvest() {

  HarvestingStats.push("harvest 0");

  if (empty()) {
    HarvestingStats.push("harvest 1: SimpleLiterals");
    revng_log(JTCountLog, "Collecting simple literals");
    for (MetaAddress PC : SimpleLiterals)
      registerJT(PC, JTReason::SimpleLiteral);
    SimpleLiterals.clear();
  }

  if (empty()) {
    HarvestingStats.push("harvest 2: SROA + InstCombine + TBDP");

    // Safely erase all unreachable blocks
    std::set<BasicBlock *> Unreachable = computeUnreachable();
    for (BasicBlock *BB : Unreachable)
      BB->dropAllReferences();
    for (BasicBlock *BB : Unreachable)
      BB->eraseFromParent();

    // TODO: move me to a commit function

    // Update the third argument of newpc calls (isJT, i.e., is this instruction
    // a jump target?)
    IRBuilder<> Builder(Context);
    Function *NewPCFunction = TheModule.getFunction("newpc");
    if (NewPCFunction != nullptr) {
      for (User *U : NewPCFunction->users()) {
        auto *Call = cast<CallInst>(U);
        if (Call->getParent() != nullptr) {
          // Report the instruction on the coverage CSV
          auto PC = MetaAddress::fromConstant(Call->getArgOperand(0));

          bool IsJT = isJumpTarget(PC);
          Call->setArgOperand(2, Builder.getInt32(static_cast<uint32_t>(IsJT)));
        }
      }
    }

    if (VerifyLog.isEnabled())
      revng_assert(not verifyModule(TheModule, &dbgs()));

    revng_log(JTCountLog, "Preliminary harvesting");

    HarvestingStats.push("InstCombine");
    legacy::FunctionPassManager OptimizingPM(&TheModule);
    OptimizingPM.add(createSROAPass());
    OptimizingPM.add(createInstSimplifyLegacyPass());
    OptimizingPM.doInitialization();
    OptimizingPM.run(*TheFunction);
    OptimizingPM.doFinalization();

    legacy::PassManager PreliminaryBranchesPM;
    PreliminaryBranchesPM.add(new TranslateDirectBranchesPass(this));
    PreliminaryBranchesPM.run(TheModule);

    if (empty()) {
      HarvestingStats.push("harvest 3: harvestWithAVI");
      revng_log(JTCountLog, "Harvesting with Advanced Value Info");
      harvestWithAVI();
    }

    // TODO: eventually, `setCFGForm` should be replaced by using a CustomCFG
    // To improve the quality of our analysis, keep in the CFG only the edges we
    // where able to recover (e.g., no jumps to the dispatcher)
    setCFGForm(CFGForm::RecoveredOnly);

    NewBranches = 0;
    legacy::PassManager AnalysisPM;
    AnalysisPM.add(new TranslateDirectBranchesPass(this));
    AnalysisPM.run(TheModule);

    // Restore the CFG
    setCFGForm(CFGForm::SemanticPreserving);

    if (JTCountLog.isEnabled()) {
      JTCountLog << std::dec << Unexplored.size() << " new jump targets and "
                 << NewBranches << " new branches were found" << DoLog;
    }
  }
}

using BWA = JumpTargetManager::BlockWithAddress;
using JTM = JumpTargetManager;
const BWA JTM::NoMoreTargets = BWA(MetaAddress::invalid(), nullptr);
