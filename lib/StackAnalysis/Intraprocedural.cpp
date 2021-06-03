/// \file Intraprocedural.cpp
/// \brief Implementation of the intraprocedural portion of the stack analysis

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <iomanip>

#include "Cache.h"
#include "InterproceduralAnalysis.h"
#include "Intraprocedural.h"

using llvm::AllocaInst;
using llvm::ArrayRef;
using llvm::BasicBlock;
using llvm::BlockAddress;
using llvm::CallInst;
using llvm::cast;
using llvm::Constant;
using llvm::ConstantInt;
using llvm::DataLayout;
using llvm::dyn_cast;
using llvm::GlobalVariable;
using llvm::Instruction;
using llvm::isa;
using llvm::LoadInst;
using llvm::Module;
using llvm::Optional;
using llvm::SmallVector;
using llvm::StoreInst;
using llvm::Type;
using llvm::UndefValue;
using llvm::UnreachableInst;
using llvm::User;

using AI = StackAnalysis::Intraprocedural::Interrupt;
using IFS = StackAnalysis::IntraproceduralFunctionSummary;

const IFS EmptyCallSummary = IFS::bottom();

// Loggers
static Logger<> SaFake("sa-fake");
static Logger<> SaTerminator("sa-terminator");
static Logger<> SaBBLog("sa-bb");

// Statistics
RunningStatistics ABIRegistersCountStats("ABIRegistersCount");
static RunningStatistics CacheHitRate("CacheHitRate");

/// \brief Per-function cache hit rate
static std::map<BasicBlock *, RunningStatistics> FunctionCacheHitRate;

/// \brief Round \p Value to \p Digits
template<typename F>
static std::string round(F Value, int Digits) {
  std::stringstream Stream;
  Stream << std::setprecision(Digits) << Value;
  return Stream.str();
}

namespace StackAnalysis {

namespace Intraprocedural {

void Analysis::initialize() {
  CacheMustHit = false;

  revng_log(SaLog, "Creating Analysis for " << getName(Entry));

  Instruction *T = Entry->getTerminator();
  revng_assert(T != nullptr);

  // Obtain the link register used to call this function
  GlobalVariable *LinkRegister = TheCache->getLinkRegister(Entry);

  // Get the register indices for for the stack pointer, the program counter
  // and the link register
  int32_t LinkRegisterIndex = 0;
  if (LinkRegister != nullptr)
    LinkRegisterIndex = TheCache->getCPUIndex(LinkRegister);
  PCIndex = TheCache->getCPUIndex(GCBI->pcReg());
  SPIndex = TheCache->getCPUIndex(GCBI->spReg());

  // Set the stack pointer to SP0+0
  ASSlot StackPointer = ASSlot::create(ASID::cpuID(), SPIndex);
  ASSlot StackSlot0 = ASSlot::create(ASID::stackID(), 0);
  InitialState = Element::initial();
  InitialState.store(Value::fromSlot(StackPointer),
                     Value::fromSlot(StackSlot0));

  // Record the slot where ther return address is stored
  ReturnAddressSlot = ((LinkRegister == nullptr) ?
                         StackSlot0 :
                         ASSlot::create(ASID::cpuID(), LinkRegisterIndex));

  if (SaLog.isEnabled()) {
    SaLog << "The return address is in ";
    if (LinkRegister != nullptr)
      SaLog << LinkRegister->getName().str();
    else
      SaLog << "the top of the stack";
    SaLog << DoLog;
  }

  TheABIIR.reset();
  IncoherentFunctions.clear();
  SuccessorsMap.clear();
  Base::initialize();
}

/// \brief Class to keep track of the Value associated to each instruction in a
///        basic block
class BasicBlockState {
public:
  using ContentMap = std::map<Instruction *, Value>;

private:
  BasicBlock *BB;
  const Module *M;
  ContentMap InstructionContent; ///< Map for the instructions in this BB
  ContentMap &VariableContent; ///< Reference to map for allocas
  const DataLayout &DL;
  const Cache *TheCache;

public:
  BasicBlockState(BasicBlock *BB,
                  ContentMap &VariableContent,
                  const DataLayout &DL,
                  const Cache *TheCache) :
    BB(BB),
    M(getModule(BB)),
    VariableContent(VariableContent),
    DL(DL),
    TheCache(TheCache) {}

  /// \brief Gets the Value associated to \p V
  ///
  /// This function handles a couple of type of llvm::Values:
  ///
  /// * AllocaInst/GlobalVariables: represent a part of the CPU state, the
  ///   result will be an ASSlot relative to the CPU address space.
  /// * Constant: represent an absolute address, the result will be an ASSlot
  ///   relative to the GLB address space with an offset equal to the actual
  ///   value of the constant.
  /// * Instruction: represents the result of a (previously analyzed)
  ///   Instruction. It can be any Value.
  Value get(llvm::Value *V) const {
    V = skipCasts(V);

    if (auto *CSV = dyn_cast<AllocaInst>(V)) {

      return Value::fromSlot(ASID::cpuID(), TheCache->getCPUIndex(CSV));

    } else if (auto *CSV = dyn_cast<GlobalVariable>(V)) {

      if (TheCache->isCPU(CSV))
        return Value::fromSlot(ASID::cpuID(), TheCache->getCPUIndex(CSV));
      else
        return Value();

    } else if (isa<UndefValue>(V)) {

      return Value();

    } else if (auto *C = dyn_cast<Constant>(V)) {

      Type *T = C->getType();
      if (T->isPointerTy() or T->getIntegerBitWidth() <= 64) {
        int32_t Offset = getZExtValue(C, DL);
        return Value::fromSlot(ASID::globalID(), Offset);
      } else {
        return Value();
      }
    }

    Instruction *I = cast<Instruction>(V);

    // I shoudl be in InstructionContent or VariableContent
    auto InstructionContentIt = InstructionContent.find(I);
    auto VariableContentIt = VariableContent.find(I);

    if (InstructionContentIt != InstructionContent.end())
      return InstructionContentIt->second;
    else if (VariableContentIt != VariableContent.end())
      return VariableContentIt->second;
    else
      revng_abort();
  }

  /// \brief Register the value of instruction \p I
  void set(Instruction *I, Value V) {
    revng_assert(I != nullptr);
    revng_assert(I->getParent() == BB,
                 "Instruction from an unexpected basic block");
    revng_assert(InstructionContent.count(I) == 0,
                 "Instruction met more than once in a basic block");

    if (SaVerboseLog.isEnabled()) {
      SaVerboseLog << "Set " << getName(I) << " to ";
      V.dump(M, SaVerboseLog);
      SaVerboseLog << DoLog;
    }

    InstructionContent[I] = V;
  }

  // TODO: this probably needs to be able to handle casts only
  /// \brief Handle automatically an otherwise un-handleable instruction
  ///
  /// This is a fallback handling of instruction not otherwise manually
  /// handled. The resulting Value will be a combination of the Values of all
  /// its operands.
  void handleGenericInstruction(Instruction *I) {
    revng_assert(I->getParent() == BB,
                 "Instruction from an unexpected basic block");

    switch (I->getOpcode()) {
    case Instruction::BitCast:
    case Instruction::IntToPtr:
    case Instruction::PtrToInt:
    case Instruction::ZExt:
    case Instruction::SExt:
      revng_assert(I->getNumOperands() == 1);
      set(I, get(I->getOperand(0)));
      break;

    default:
      set(I, Value::empty());
      break;
    }
  }

  /// \brief Compute the set of BasicBlocks affected by changes in the current
  ///        one
  std::set<BasicBlock *> computeAffected() {
    std::set<BasicBlock *> Result;
    for (auto &P : InstructionContent) {
      Instruction *I = P.first;
      Value &NewValue = P.second;

      revng_assert(I->getParent() == BB);

      if (I->isUsedOutsideOfBlock(BB)) {
        bool Changed = false;

        // Has this instruction ever been registered?
        auto It = VariableContent.find(I);
        if (It == VariableContent.end()) {
          VariableContent[I] = NewValue;
          Changed = true;
        } else {
          // If not, are we saying something new?
          Value &OldValue = It->second;
          if (not NewValue.lowerThanOrEqual(OldValue)) {
            OldValue = NewValue;
            Changed = true;
          }
        }

        if (Changed) {
          for (User *U : I->users()) {
            if (auto *UserI = dyn_cast<Instruction>(U)) {
              BasicBlock *UserBB = UserI->getParent();
              if (UserBB != BB)
                Result.insert(UserBB);
            }
          }
        }
      }
    }

    return Result;
  }
};

static llvm::Value *getModifyAndReassign(Instruction *I) {
  auto *Load = dyn_cast<LoadInst>(I->getOperand(0));
  if (Load == nullptr)
    return nullptr;

  for (User *U : I->users()) {
    auto *Store = dyn_cast<StoreInst>(U);
    if (Store != nullptr and Store->getValueOperand() == I
        and Load->getPointerOperand() == Store->getPointerOperand()) {
      return Load->getPointerOperand();
    }
  }

  return nullptr;
}

Interrupt Analysis::transfer(BasicBlock *BB) {
  auto SP0 = ASID::stackID();

  BlockType::Values Type = GCBI->getType(BB);
  revng_assert(Type != BlockType::AnyPCBlock
               and Type != BlockType::UnexpectedPCBlock);

  // Create a copy of the initial state associated to this basic block
  auto It = State.find(BB);
  revng_assert(It != State.end());
  Element Result = It->second.copy();

  revng_log(SaBBLog, "Analyzing " << getName(BB));
  LoggerIndent<> Y(SaBBLog);

  if (SaLog.isEnabled()) {
    SaLog << "Analyzing basic block " << getName(BB) << DoLog;
    Result.dump(M, SaLog);
    SaLog << DoLog;
  }

  // Reset the basic ABI IR basic block
  ABIIRBasicBlock &ABIBB = TheABIIR.get(BB);
  ABIBB.clear();

  // TODO: prune all the info about dead instructions

  // Initialize an object to keep track of the values associated to each
  // instruction in the current basic block
  BasicBlockState BBState(BB, VariableContent, M->getDataLayout(), TheCache);

  for (Instruction &I : *BB) {

    revng_log(SaVerboseLog, "NewInstruction: " << getName(&I));

    switch (I.getOpcode()) {
    case Instruction::Load: {
      auto *Load = cast<LoadInst>(&I);

      // Get the value associated to the pointer operand and load from it from
      // Result
      const Value &AddressValue = BBState.get(Load->getPointerOperand());
      BBState.set(&I, Result.load(AddressValue));

      // If it's not an identity load and we're loading from a register or the
      // stack, register the load in the ABI IR
      if (not TheCache->isIdentityLoad(Load)) {
        if (const ASSlot *Target = AddressValue.directContent()) {
          if (isCSV(*Target) or Target->addressSpace() == SP0)
            ABIBB.append(ABIIRInstruction::createLoad(*Target));
        }
      }

    } break;

    case Instruction::Store: {
      auto *Store = cast<StoreInst>(&I);

      // Completely ignore identity stores
      if (TheCache->isIdentityStore(Store))
        break;

      // Update slot Address in Result with StoredValue
      Value Address = BBState.get(Store->getPointerOperand());
      Value StoredValue = BBState.get(Store->getValueOperand());
      Result.store(Address, StoredValue);

      // If we're loading from a register or the stack register the store in
      // the ABI IR
      if (const ASSlot *Target = Address.directContent())
        if (isCSV(*Target) or Target->addressSpace() == SP0)
          ABIBB.append(ABIIRInstruction::createStore(*Target));

    } break;

    case Instruction::And: {
      // If we're masking an address with a mask that is at most as strict as
      // the one for instruction alignment, ignore the operation. This allows
      // us to correctly track value whose lower bits are suppressed before
      // being written to the PC.
      // Note that this works if the address is pointing to code, but not
      // necessarily if it's pointing to data.
      Value FirstOperand = BBState.get(I.getOperand(0));
      if (auto *SecondOperand = dyn_cast<ConstantInt>(I.getOperand(1))) {
        uint64_t Mask = getSignedLimitedValue(SecondOperand);
        uint64_t Flip = ~Mask + 1;
        bool IsContiguousMask = Flip and not(Flip & (Flip - 1));

        if (IsContiguousMask) {
          bool Forward = false;

          // Forward any contiguous mask applied to the stack pointer, it's
          // likely stack alignment
          llvm::Value *Pointer = getModifyAndReassign(&I);
          if (Pointer != nullptr and GCBI->isSPReg(Pointer)) {
            Forward = true;
          } else {
            uint64_t SignificantPCBits;
            if (GCBI->pcRegSize() == 4) {
              SignificantPCBits = std::numeric_limits<uint32_t>::max();
            } else {
              revng_assert(GCBI->pcRegSize() == 8);
              SignificantPCBits = std::numeric_limits<uint64_t>::max();
            }
            uint64_t AlignmentMask = GCBI->instructionAlignment() - 1;
            SignificantPCBits = SignificantPCBits & ~AlignmentMask;

            Forward = (SignificantPCBits & Mask) == SignificantPCBits;
          }

          if (Forward) {
            BBState.set(&I, FirstOperand);
            break;
          }
        }
      }

      // In all other cases, treat it as a regular instruction
      BBState.handleGenericInstruction(&I);
    } break;

    case Instruction::Add:
    case Instruction::Sub: {
      int Sign = (I.getOpcode() == Instruction::Add) ? +1 : -1;

      // If the second operand is constant we can handle it
      Value FirstOperand = BBState.get(I.getOperand(0));
      if (auto *Addend = dyn_cast<ConstantInt>(I.getOperand(1))) {
        if (FirstOperand.add(Sign * getLimitedValue(Addend))) {
          BBState.set(&I, FirstOperand);
          break;
        }
      }

      // In all other cases, treat it as a regular instruction
      BBState.handleGenericInstruction(&I);
    } break;

    case Instruction::Call: {
      auto *Call = cast<CallInst>(&I);

      // If the call returns something, introduce a dummy value in BBState
      if (not Call->getFunctionType()->getReturnType()->isVoidTy())
        BBState.set(&I, Value::empty());

      FunctionCall Indirect(nullptr, &I);

      const llvm::Function *Callee = getCallee(&I);
      revng_assert(Callee != nullptr);
      // We should have function calls to helpers, markers, abort or
      // intrinsics. Assert in other cases.
      revng_assert(isCallToHelper(&I) || isMarker(&I)
                   || Callee->getName() == "abort" || Callee->isIntrinsic());

      if (isCallToHelper(&I)) {

        // Compute the stack size for the call to the helper
        Optional<int32_t> CallerStackSize = stackSize(Result);

        // Register the call site (as an indirect call) along with the current
        // stack size
        registerStackSizeAtCallSite(Indirect, CallerStackSize);

        // Create in the ABIIR a load for each read register and a store for
        // each written register
        auto UsedCSVs = GeneratedCodeBasicInfo::getCSVUsedByHelperCall(Call);

        for (GlobalVariable *CSV : UsedCSVs.Read)
          if (TheCache->isCSV(CSV))
            ABIBB.append(ABIIRInstruction::createLoad(slotFromCSV(CSV)));

        for (GlobalVariable *CSV : UsedCSVs.Written)
          if (TheCache->isCSV(CSV))
            ABIBB.append(ABIIRInstruction::createStore(slotFromCSV(CSV)));
      }

    } break;

    case Instruction::Br:
    case Instruction::Switch: {

      // We're at the end of the basic block, handleTerminator will provide us
      // an Interrupt to forward back
      Interrupt BBResult = handleTerminator(&I, Result, ABIBB);

      // Register all the successors in the ABI IR too
      if (BBResult.hasSuccessors())
        for (BasicBlock *BB : BBResult)
          ABIBB.addSuccessor(&TheABIIR.get(BB));

      // Record the type of this branch
      BranchesType[BB] = BBResult.type();

      // Re-enqueue for analysis all the basic block affected by changes in
      // the current one
      std::set<BasicBlock *> ToReanalyze = BBState.computeAffected();
      for (BasicBlock *BB : ToReanalyze)
        if (GCBI->getType(BB) != BlockType::IndirectBranchDispatcherHelperBlock)
          registerToVisit(BB);

      if (SaLog.isEnabled()) {
        SaLog << "Basic block terminated: " << getName(BB) << "\n";
        BBResult.dump(M, SaLog);
        SaLog << DoLog;
      }

      return BBResult;
    }

    case Instruction::Unreachable:
      return AI::create(std::move(Result), BranchType::Unreachable);

    default:
      BBState.handleGenericInstruction(&I);
      break;
    }

    revng_assert(Result.verify());
  }

  revng_abort();
}

static SmallVector<BasicBlock *, 2>
directSuccessors(GeneratedCodeBasicInfo *GCBI, Instruction *T) {
  revng_assert(T->isTerminator());
  SmallVector<BasicBlock *, 2> Successors;
  for (BasicBlock *Successor : llvm::successors(T)) {
    BlockType::Values SuccessorType = GCBI->getType(Successor);
    if (SuccessorType != BlockType::UnexpectedPCBlock
        and SuccessorType != BlockType::AnyPCBlock)
      Successors.push_back(Successor);
  }
  return Successors;
}

Interrupt Analysis::handleTerminator(Instruction *T,
                                     Element &Result,
                                     ABIIRBasicBlock &ABIBB) {
  namespace BT = BranchType;
  BasicBlock *BB = T->getParent();
  FakeReturns.erase(BB);

  revng_assert(T->isTerminator());
  revng_assert(not isa<UnreachableInst>(T));

  LogOnReturn<> X(SaTerminator);
  SaTerminator << T;

  Value StackPointer = Value::fromSlot(ASID::cpuID(), SPIndex);

  bool HasUnknownStackSize = not Result.load(StackPointer).hasDirectContent();
  if (HasUnknownStackSize)
    SaTerminator << " UnknownStackSize";

  // 0. Check if it's a direct killer basic block
  // TODO: we should move the metadata enums and functions to get their names to
  //       GCBI
  // TODO: this is likely wrong
  if (GCBI->isKiller(T)
      and GCBI->getKillReason(T) != KillReason::LeadsToKiller) {
    SaTerminator << " Killer";
    return AI::create(std::move(Result), BT::Killer);
  }

  // 1. Check if we're dealing with instruction-local control flow (e.g., the if
  //    generated due to a conditional move)
  // 2. Check if it's an indirect branch, which means that "anypc" is among
  //    its successors
  bool IsInstructionLocal = false;
  bool IsIndirect = false;
  bool IsUnresolvedIndirect = false;
  bool JustUnexpected = true;

  for (BasicBlock *Successor : llvm::successors(T)) {
    BlockType::Values SuccessorType = GCBI->getType(Successor->getTerminator());

    // If at least one successor is not a jump target, the branch is instruction
    // local
    namespace BT = BlockType;
    constexpr auto IBDHB = BT::IndirectBranchDispatcherHelperBlock;
    IsInstructionLocal = (IsInstructionLocal
                          or SuccessorType == BT::TranslatedBlock
                          or SuccessorType == IBDHB);

    revng_assert(SuccessorType != BT::RootDispatcherBlock);

    IsIndirect = (IsIndirect or SuccessorType == BT::AnyPCBlock
                  or SuccessorType == BT::UnexpectedPCBlock
                  or SuccessorType == IBDHB);

    IsUnresolvedIndirect = (IsUnresolvedIndirect
                            or SuccessorType == BT::AnyPCBlock);

    JustUnexpected = JustUnexpected and SuccessorType == BT::UnexpectedPCBlock;
  }

  if (IsIndirect)
    SaTerminator << " IsIndirect";

  if (IsUnresolvedIndirect)
    SaTerminator << " IsUnresolvedIndirect";

  if (IsInstructionLocal) {
    SaTerminator << " IsInstructionLocal";

    return AI::createWithSuccessors(std::move(Result),
                                    BT::InstructionLocalCFG,
                                    directSuccessors(GCBI, T));
  }

  // 3. Check if this a function call (although the callee might not be a proper
  //    function)
  bool IsFunctionCall = false;
  BasicBlock *Callee = nullptr;
  BasicBlock *ReturnFromCall = nullptr;
  MetaAddress ReturnAddress = MetaAddress::invalid();

  if (CallInst *Call = getFunctionCall(T->getParent())) {
    IsFunctionCall = true;
    auto *Arg0 = Call->getArgOperand(0);
    auto *Arg1 = Call->getArgOperand(1);
    auto *Arg2 = Call->getArgOperand(2);

    if (auto *CalleeBlockAddress = dyn_cast<BlockAddress>(Arg0))
      Callee = CalleeBlockAddress->getBasicBlock();

    auto *ReturnBlockAddress = cast<BlockAddress>(Arg1);
    ReturnFromCall = ReturnBlockAddress->getBasicBlock();

    ReturnAddress = MetaAddress::fromConstant(Arg2);

    SaTerminator << " IsFunctionCall (callee " << Callee << ", return "
                 << ReturnFromCall << ")";
  }

  // 4. Check if the stack pointer is in position valid for returning

  // Get the current value of the stack pointer
  // TODO: we should evaluate the approximation introduced here appropriately
  Value StackPointerValue = Result.load(StackPointer);

  const ASSlot *StackPointerSlot = StackPointerValue.directContent();

  auto SP0 = ASID::stackID();
  bool IsReadyToReturn = (StackPointerSlot != nullptr
                          and StackPointerSlot->addressSpace() == SP0
                          and StackPointerSlot->offset() >= 0);

  if (IsReadyToReturn)
    SaTerminator << " IsReadyToReturn";

  // 5. Are we jumping to the return address? Are we jumping to the return
  //    address from a fake function?
  bool IsReturn = false;
  bool IsReturnFromFake = false;
  uint64_t FakeFunctionReturnAddress = 0;

  if (IsIndirect) {
    // Get the current value being stored in the program counter
    Value ProgramCounter = Value::fromSlot(ASID::cpuID(), PCIndex);
    Value ProgramCounterValue = Result.load(ProgramCounter);

    const ASSlot *PCContent = ProgramCounterValue.directContent();
    const ASSlot *PCTag = ProgramCounterValue.tag();

    // It's a return if the PC has a value with a name matching the name of the
    // initial value of the link register
    IsReturn = (PCTag != nullptr) and (*PCTag == ReturnAddressSlot);

    if (SaTerminator.isEnabled()) {

      if (IsReturn) {
        SaTerminator << " ReturnsToLinkRegister";
      } else {
        SaTerminator << " (";

        ReturnAddressSlot.dump(M, SaTerminator);
        SaTerminator << " != ";
        if (PCTag == nullptr)
          SaTerminator << "nullptr";
        else
          PCTag->dump(M, SaTerminator);
        SaTerminator << ")";
      }
    }

    if (PCContent != nullptr) {
      // Check if it's a return from fake
      if (!IsReturn) {
        if (PCContent->addressSpace() == ASID::globalID()) {
          uint64_t Offset = PCContent->offset();
          FakeFunctionReturnAddress = Offset;
          IsReturnFromFake = FakeReturnAddresses.count(Offset) != 0;
        }
      }
    }
  }

  if (IsReturnFromFake)
    SaTerminator << " IsReturnFromFake";

  // 6. Using the collected information, classify the branch type

  // Are we returning to the return address?
  if (IsReturn) {
    // This looks like an actual return
    insert_or_assign(ReturnCandidates, T->getParent(), Result.copy());
    return AI::create(std::move(Result), BT::Return);
  }

  if (IsFunctionCall)
    return handleCall(T, Callee, ReturnAddress, ReturnFromCall, Result, ABIBB);

  // Is it an indirect jump?
  if (IsIndirect) {

    // Is it targeting an address that we registered as a return from fake
    // function call?
    if (IsReturnFromFake) {
      // Continue from there
      MetaAddress MA = GCBI->fromPC(FakeFunctionReturnAddress);
      FakeReturns.insert({ BB, MA });
      BasicBlock *ReturnBB = GCBI->getBlockAt(MA);
      return AI::createWithSuccessor(std::move(Result),
                                     BT::FakeFunctionReturn,
                                     ReturnBB);
    }

    // Check if it's a real indirect jump, i.e. we're not 100% of the targets
    if (IsUnresolvedIndirect or JustUnexpected) {
      if (IsReadyToReturn) {
        // If the stack is not in a valid position, we consider it an indirect
        // tail call
        return handleCall(T,
                          nullptr,
                          MetaAddress::invalid(),
                          nullptr,
                          Result,
                          ABIBB);
      } else {
        // We have an indirect jump with a stack not ready to return: it's a
        // longjmp
        return AI::create(std::move(Result), BT::LongJmp);
      }
    }
  }

  SaTerminator << " FunctionLocalCFG";
  return AI::createWithSuccessors(std::move(Result),
                                  BT::FunctionLocalCFG,
                                  directSuccessors(GCBI, T));
}

std::pair<FunctionType::Values, Element> Analysis::finalize() {
  MetaAddress EntryPC = getPC(Entry->getTerminator()).first;

#ifndef NDEBUG
  // Compute the set of reachable basic blocks
  llvm::ReversePostOrderTraversal<ABIIRBasicBlock *> RPOT(TheABIIR.entry());
  std::set<BasicBlock *> Reachable;
  for (ABIIRBasicBlock *Block : RPOT)
    Reachable.insert(Block->basicBlock());
#endif

  //
  // Return SP election
  //

  // Combine the value of the stack pointer of each candidate return to see if
  // they agree.  Meanwhile find the return that is closest (but after) the
  // entry point and that has a valid stack size
  Value BestSP;
  uint64_t ClosestPC = EntryPC.address() - 1;
  bool First = true;
  Value Combined;
  for (auto &P : ReturnCandidates) {
    BasicBlock *BB = P.first;
    const Element &Result = P.second;
    MetaAddress PC = getPC(BB->getTerminator()).first;
    uint64_t PCAddress = PC.address();

#ifndef NDEBUG
    revng_assert(Reachable.count(BB) != 0);
#endif

    Value StackPointer = Value::fromSlot(ASID::cpuID(), SPIndex);
    Value StackPointerValue = Result.load(StackPointer);

    if (First) {
      Combined = StackPointerValue;
      First = false;
    } else {
      Combined.combine(StackPointerValue);
    }

    if (const ASSlot *Slot = StackPointerValue.directContent()) {
      if (PC.addressGreaterThanOrEqual(EntryPC) and PCAddress < ClosestPC
          and Slot->addressSpace() == ASID::stackID() and Slot->offset() >= 0) {
        ClosestPC = PCAddress;
        BestSP = StackPointerValue;
      }
    }
  }

  // Do they all agree on a fixed stack pointer?
  if (const ASSlot *Slot = Combined.directContent()) {
    if (Slot->addressSpace() == ASID::stackID()) {
      if (Slot->offset() >= 0) {
        BestSP = Combined;
      } else {
        // Every return agrees the stack has grown: it's a fake function, let's
        // inline it
        return { FunctionType::Fake, Element::bottom() };
      }
    }
  }

  if (not BestSP.hasDirectContent())
    return { FunctionType::NoReturn, Element::bottom() };

  // Combine all the values of the non-broken returns, mark as broken all the
  // others
  First = true;
  Element GrandResult = Element::bottom();
  for (auto &P : ReturnCandidates) {
    BasicBlock *BB = P.first;
    Element &ReturnResult = P.second;

    Value StackPointer = Value::fromSlot(ASID::cpuID(), SPIndex);
    Value StackPointerValue = ReturnResult.load(StackPointer);

    if (BestSP.hasDirectContent() and StackPointerValue == BestSP) {
      // Mark as return basic block in the ABI IR
      TheABIIR.get(BB).setReturn();

      // OK, we're compatible, make ReturnResult part of the final result
      if (First) {
        GrandResult = std::move(ReturnResult);
        First = false;
      } else {
        GrandResult.combine(std::move(ReturnResult));
      }
    } else {
      // Mark as broken
      auto &Type = BranchesType[BB];
      if (Type == BranchType::Return)
        Type = BranchType::BrokenReturn;
      else if (Type == BranchType::IndirectTailCall)
        Type = BranchType::LongJmp;
    }
  }

  return { FunctionType::Regular, std::move(GrandResult) };
}

Interrupt Analysis::handleCall(Instruction *Caller,
                               BasicBlock *Callee,
                               MetaAddress ReturnAddress,
                               BasicBlock *ReturnFromCall,
                               Element &Result,
                               ABIIRBasicBlock &ABIBB) {
  namespace BT = BranchType;

  revng_assert(Callee == nullptr or getName(Callee) != "unexpectedpc");

  const bool IsRecursive = InProgressFunctions.count(Callee) != 0;
  const bool IsIndirect = (Callee == nullptr);
  const bool IsIndirectTailCall = IsIndirect and (ReturnFromCall == nullptr);
  bool IsKiller = false;
  bool ABIOnly = false;

  FunctionCall TheFunctionCall = { Callee, Caller };
  int32_t PCRegSize = GCBI->pcRegSize();

  Value StackPointer = Value::fromSlot(ASID::cpuID(), SPIndex);
  Value OldStackPointer = Result.load(StackPointer);
  Value PC = Value::fromSlot(ASID::cpuID(), PCIndex);

  // Handle special function types:
  //
  // 1. Calls to Fake functions will be inlined.
  // 2. Calls to NoReturn functions will make the current basic block a Killer
  // 3. Calls to IndirectTailCall functions are considered as indirect function
  //    calls
  if (not IsIndirect) {
    if (TheCache->isFakeFunction(Callee)) {
      // Make sure the CacheMustHit bit is turned off
      resetCacheMustHit();

      SaTerminator << " IsFakeFunctionCall";
      // Assume normal control flow (i.e., inline)
      FakeReturnAddresses.insert(ReturnAddress.asPC());
      return AI::createWithSuccessor(std::move(Result),
                                     BT::FakeFunctionCall,
                                     Callee);
    } else if (TheCache->isNoReturnFunction(Callee)) {
      SaTerminator << " IsNoReturnFunction";
      ABIOnly = true;
    }
  }

  // If we know the current stack frame size, copy the arguments
  Optional<int32_t> CallerStackSize = stackSize(Result);

  const IFS *CallSummary = &EmptyCallSummary;

  revng_assert(not(IsRecursive && IsIndirect));

  // Is it an direct function call?
  if (not IsIndirect) {
    // We have a direct call
    revng_assert(Callee != nullptr);

    // It's a direct function call, lookup the <Callee, Context> pair in the
    // cache
    Optional<const IFS *> CacheEntry;
    CacheEntry = TheCache->get(Callee);

    if (not CacheMustHit) {
      const char *ResultString = nullptr;
      if (CacheEntry) {
        CacheHitRate.push(1);
        FunctionCacheHitRate[Callee].push(1);
        ResultString = "hit";
      } else {
        CacheHitRate.push(0);
        FunctionCacheHitRate[Callee].push(0);
        ResultString = "miss";
      }

      if (SaInterpLog.isEnabled()) {
        SaInterpLog << "Cache " << ResultString << " for " << Callee << " at "
                    << Caller << " (";
        auto Mean = FunctionCacheHitRate[Callee].mean();
        SaInterpLog << "function hit rate: " << round(100 * Mean, 4) << "%";
        SaInterpLog << ", hit rate: " << round(100 * CacheHitRate.mean(), 4)
                    << "%) ";
        SaInterpLog << DoLog;
      }
    }

    // Do we have a cache hit?
    // If we don't we return control the interprocedural part, and we record
    // that next time we *must* have a cache hit. If we don't there's the risk
    // we're going to loop endlessly.
    if (CacheEntry) {
      resetCacheMustHit();

      // We have a match in the cache
      CallSummary = *CacheEntry;
    } else {
      // Ensure we don't get a cache miss twice in a row
      revng_assert(not CacheMustHit);

      // Next time the cache will have to hit
      CacheMustHit = true;

      // We don't have a match in the cache. Ask interprocedural analysis to
      // analyze this function call with the current context
      return AI::createUnhandledCall(Callee);
    }

  } // not IsIndirect

  // If we got to this point, we now have a cached result of what the callee
  // does. Let's apply it.

  if (SaLog.isEnabled()) {
    SaLog << "The summary result for a call to " << getName(Callee) << " is\n";
    CallSummary->dump(M, SaLog);
    SaLog << DoLog;
  }

  if (not ABIOnly and not CallSummary->FinalState.isBottom()) {
    // Use the summary from the cache
    Result.apply(CallSummary->FinalState);
  }

  if (IsRecursive or IsIndirect) {
    ABIBB.append(ABIIRInstruction::createIndirectCall(TheFunctionCall));
  } else {
    std::set<int32_t> StackArguments;
    if (CallerStackSize and *CallerStackSize >= 0)
      StackArguments = CallSummary->FinalState.stackArguments(*CallerStackSize);
    ABIBB.append(ABIIRInstruction::createDirectCall(TheFunctionCall,
                                                    CallSummary->ABI.copy(),
                                                    StackArguments));
  }

  // Record frame size
  registerStackSizeAtCallSite(TheFunctionCall, CallerStackSize);

  // Resume the analysis from where we left off

  // Restore the stack pointer
  GlobalVariable *CalleeLinkRegister = TheCache->getLinkRegister(Callee);
  if (CalleeLinkRegister == nullptr) {
    // Increase the stack pointer of the size of the PC reg
    OldStackPointer.add(PCRegSize);
  }
  Result.store(StackPointer, OldStackPointer);

  // Restore the PC
  // TODO: handle return address from indirect tail calls
  ASSlot ReturnAddressSlot = ASSlot::create(ASID::globalID(),
                                            ReturnAddress.asPCOrZero());
  Result.store(PC, Value::fromSlot(ReturnAddressSlot));

  revng_assert(not(IsIndirectTailCall and IsKiller));
  if (IsIndirectTailCall) {
    // We consider indirect tail calls as returns
    insert_or_assign(ReturnCandidates, Caller->getParent(), Result.copy());
    return AI::create(std::move(Result), BT::IndirectTailCall);
  } else if (IsKiller) {
    return AI::create(std::move(Result), BT::Killer);
  } else {
    revng_assert(ReturnFromCall != nullptr);
    auto Reason = IsIndirect ? BT::IndirectCall : BT::HandledCall;
    return AI::createWithSuccessor(std::move(Result), Reason, ReturnFromCall);
  }
}

ASSlot Analysis::slotFromCSV(llvm::User *U) const {
  return ASSlot::create(ASID::cpuID(), TheCache->getCPUIndex(U));
}

IFS Analysis::createSummary() {
  auto P = finalize();
  FunctionType::Values Type = P.first;
  Element GrandResult = std::move(P.second);

  // Fake functions need no further analysis (NoReturn functions do)
  if (Type == FunctionType::Fake)
    return IFS::createFake();

  // Finalize the ABI IR (e.g., fill-in reverse links)
  TheABIIR.finalize();

  FunctionABI ABI;

  if (SaABI.isEnabled()) {
    revng_log(SaABI, "Starting analysis of " << Entry);
    TheABIIR.dump(SaABI, M);
    SaABI << DoLog;
  }

  revng_assert(TheABIIR.verify(), "The ABI IR is invalid");

  // Run the almighty ABI analyses
  ABI.analyze(TheABIIR);

  // Find all the function calls that lead to results incoherent with the
  // callees and register them

  std::set<int32_t> WrittenRegisters = TheABIIR.writtenRegisters();

  IFS Summary;
  if (Type == FunctionType::Regular) {
    Summary = IFS::createRegular(std::move(GrandResult),
                                 std::move(ABI),
                                 std::move(FrameSizeAtCallSite),
                                 std::move(BranchesType),
                                 std::move(WrittenRegisters),
                                 std::move(FakeReturns));
  } else {
    Summary = IFS::createNoReturn(std::move(ABI),
                                  std::move(FrameSizeAtCallSite),
                                  std::move(BranchesType),
                                  std::move(WrittenRegisters),
                                  std::move(FakeReturns));
  }
  findIncoherentFunctions(Summary);

  if (SaABI.isEnabled()) {
    SaABI << "ABI analyses on " << Entry << " completed:\n";
    Summary.dump(M, SaABI);
    SaABI << DoLog;
  }

  return Summary;
}

void Analysis::findIncoherentFunctions(const IFS &ABISummary) {
  // TODO: do we need to take into account also all the registers used
  //       in the various function calls?
  const IFS::LocalSlotVector &Slots = ABISummary.LocalSlots;

  for (const FunctionCall &FC : TheABIIR.incoherentCalls()) {
    revng_log(SaFake,
              FC.callee() << " (" << FC.callInstruction() << ") is fake.");
    IncoherentFunctions.insert(FC.callee());
  }

  // Loop over all the function calls in this function
  for (const auto &P : FrameSizeAtCallSite) {
    const FunctionCall TheFunctionCall = P.first;
    BasicBlock *Callee = TheFunctionCall.callee();

    // We cannot perform any coherency check on indirect function calls
    if (Callee == nullptr)
      continue;

    // TODO: this is an hack, functions marked as fake should somehow be
    //       purged from CallsContext
    if (TheCache->isFakeFunction(Callee))
      continue;

    // We might not have an entry, e.g., if they callee is noreturn
    Optional<const IFS *> Cache = TheCache->get(Callee);
    if (Cache) {
      const FunctionABI &CalleeSummary = (*Cache)->ABI;

      // Loop over all the slots being considered in this function
      for (auto &Slot : Slots) {
        if (not isCoherent(ABISummary.ABI,
                           CalleeSummary,
                           TheFunctionCall,
                           Slot)) {
          IncoherentFunctions.insert(Callee);
          break;
        }
      }
    }
  }
}

bool Analysis::isCoherent(const FunctionABI &CallerSummary,
                          const FunctionABI &CalleeSummary,
                          FunctionCall TheFunctionCall,
                          IFS::LocalSlot Slot) const {
  int32_t Offset = Slot.first.offset();
  BasicBlock *Callee = TheFunctionCall.callee();

  switch (Slot.second) {
  case LocalSlotType::UsedRegister: {
    FunctionRegisterArgument FunctionArgument;
    FunctionCallRegisterArgument FunctionCallArgument;
    CalleeSummary.applyResults(FunctionArgument, Offset);
    CallerSummary.applyResults(FunctionCallArgument, TheFunctionCall, Offset);
    FunctionRegisterArgument CombinedArgument = FunctionArgument;
    CombinedArgument.combine(FunctionCallArgument);

    if (CombinedArgument.isContradiction()) {

      if (SaFake.isEnabled()) {
        SaFake << "Contradiction at ";
        TheFunctionCall.dump(SaFake);
        SaFake << " on argument ";
        ASSlot::create(ASID::cpuID(), Offset).dump(M, SaFake);
        SaFake << ": caller says is ";
        FunctionCallArgument.dump(SaFake);
        SaFake << ", while callee says is ";
        FunctionArgument.dump(SaFake);
        SaFake << ", marking " << Callee << " as fake." << DoLog;
      }

      return false;
    }

    FunctionReturnValue TheFunctionReturnValue;
    FunctionCallReturnValue TheFunctionCallReturnValue;
    CalleeSummary.applyResults(TheFunctionReturnValue, Offset);
    CallerSummary.applyResults(TheFunctionCallReturnValue,
                               TheFunctionCall,
                               Offset);
    FunctionReturnValue CombinedReturnValue = TheFunctionReturnValue;
    CombinedReturnValue.combine(TheFunctionCallReturnValue);

    if (CombinedReturnValue.isContradiction()) {

      if (SaFake.isEnabled()) {
        SaFake << "Contradiction at ";
        TheFunctionCall.dump(SaFake);
        SaFake << " on return value ";
        ASSlot::create(ASID::cpuID(), Offset).dump(M, SaFake);
        SaFake << ": caller says is ";
        TheFunctionCallReturnValue.dump(SaFake);
        SaFake << ", while callee says is ";
        TheFunctionReturnValue.dump(SaFake);
        SaFake << ", marking " << Callee << " as fake." << DoLog;
      }

      return false;
    }

  } break;

  case LocalSlotType::ForwardedArgument:
  case LocalSlotType::ForwardedReturnValue:
  case LocalSlotType::ExplicitlyCalleeSavedRegister:
    break;
  }

  return true;
}

} // namespace Intraprocedural

} // namespace StackAnalysis
