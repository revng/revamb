#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <map>
#include <set>
#include <vector>

#include "boost/icl/interval_map.hpp"
#include "boost/icl/interval_set.hpp"
#include "boost/type_traits/is_same.hpp"

#include "llvm/ADT/Optional.h"
#include "llvm/IR/Instructions.h"

#include "revng/BasicAnalyses/MaterializedValue.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/ProgramCounterHandler.h"
#include "revng/Support/revng.h"

#include "BinaryFile.h"

// Forward declarations
namespace llvm {
class BasicBlock;
class Function;
class Instruction;
class LLVMContext;
class Module;
class SwitchInst;
class StoreInst;
class Value;
} // namespace llvm

class JumpTargetManager;
class ProgramCounterHandler;

template<typename Map>
typename Map::const_iterator
containing(Map const &m, typename Map::key_type const &k) {
  typename Map::const_iterator it = m.upper_bound(k);
  if (it != m.begin()) {
    return --it;
  }
  return m.end();
}

template<typename Map>
typename Map::iterator containing(Map &m, typename Map::key_type const &k) {
  typename Map::iterator it = m.upper_bound(k);
  if (it != m.begin()) {
    return --it;
  }
  return m.end();
}

/// \brief Transform constant writes to the PC in jumps
///
/// This pass looks for all the calls to the `ExitTB` function calls, looks for
/// the last write to the PC before them, checks if the written value is
/// statically known, and, if so, replaces it with a jump to the corresponding
/// translated code. If the write to the PC is not constant, no action is
/// performed, and the call to `ExitTB` remains there for later handling.
class TranslateDirectBranchesPass : public llvm::ModulePass {
public:
  TranslateDirectBranchesPass() :
    llvm::ModulePass(ID), JTM(nullptr), PCH(nullptr) {}

  TranslateDirectBranchesPass(JumpTargetManager *JTM);

  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override;

  bool runOnModule(llvm::Module &M) override;

private:
  /// \brief Remove all the constant writes to the PC
  bool pinConstantStore(llvm::Function &F);

  /// \brief Pin PC-stores for which AVI provided useful results
  bool pinAVIResults(llvm::Function &F);

  /// Introduces a fallthrough branch if there's no store to PC before the last
  /// call to an helper
  ///
  /// \return true if the \p Call has been handled (i.e. a fallthrough jump has
  ///         been inserted.
  bool forceFallthroughAfterHelper(llvm::CallInst *Call);

  void pinExitTB(llvm::CallInst *ExitTBCall,
                 ProgramCounterHandler::DispatcherTargets &Destinations);

  void
  pinConstantStoreInternal(MetaAddress Address, llvm::CallInst *ExitTBCall);

public:
  static char ID;

private:
  JumpTargetManager *JTM;
  ProgramCounterHandler *PCH;
};

namespace CFGForm {

/// \brief Possible forms the CFG we're building can assume.
///
/// Generally the CFG should stay in the SemanticPreserving state, but it
/// can be temporarily changed to make certain analysis (e.g., computation of
/// the dominator tree) more effective for certain purposes.
enum Values {
  /// The CFG is an unknown state
  UnknownForm,

  /// The dispatcher jumps to all the jump targets, and all the indirect jumps
  /// go to the dispatcher
  SemanticPreserving,

  /// The dispatcher only jumps to jump targets without other predecessors and
  /// indirect jumps do not go to the dispatcher, but to an unreachable
  /// instruction
  RecoveredOnly,

  /// Similar to RecoveredOnly, but all jumps forming a function call are
  /// converted to jumps to the return address
  NoFunctionCalls
};

inline const char *getName(Values V) {
  switch (V) {
  case UnknownForm:
    return "UnknownForm";
  case SemanticPreserving:
    return "SemanticPreserving";
  case RecoveredOnly:
    return "RecoveredOnly";
  case NoFunctionCalls:
    return "NoFunctionCalls";
  }

  revng_abort();
}

} // namespace CFGForm

class CPUStateAccessAnalysisPass;

class JumpTargetManager {
private:
  using interval_set = boost::icl::interval_set<MetaAddress, compareAddress>;
  using interval = boost::icl::interval<MetaAddress, compareAddress>;
  using MetaAddressSet = std::set<MetaAddress>;

public:
  using BlockWithAddress = std::pair<MetaAddress, llvm::BasicBlock *>;
  static const BlockWithAddress NoMoreTargets;

  class JumpTarget {
  public:
    JumpTarget() : BB(nullptr), Reasons(0) {}
    JumpTarget(llvm::BasicBlock *BB) : BB(BB), Reasons(0) {}
    JumpTarget(llvm::BasicBlock *BB, JTReason::Values Reason) :
      BB(BB), Reasons(static_cast<uint32_t>(Reason)) {}

    llvm::BasicBlock *head() const { return BB; }
    bool hasReason(JTReason::Values Reason) const {
      return (Reasons & static_cast<uint32_t>(Reason)) != 0;
    }
    void setReason(JTReason::Values Reason) {
      Reasons |= static_cast<uint32_t>(Reason);
    }
    uint32_t getReasons() const { return Reasons; }

    bool isOnlyReason(JTReason::Values Reason) const {
      return (hasReason(Reason)
              and (Reasons & ~static_cast<uint32_t>(Reason)) == 0);
    }

    std::vector<const char *> getReasonNames() const {
      std::vector<const char *> Result;

      uint32_t LastReason = static_cast<uint32_t>(JTReason::LastReason);
      for (unsigned Reason = 1; Reason <= LastReason; Reason <<= 1) {
        JTReason::Values R = static_cast<JTReason::Values>(Reason);
        if (hasReason(R))
          Result.push_back(JTReason::getName(R));
      }

      return Result;
    }

    std::string describe() const {
      std::stringstream SS;
      SS << getName(BB) << ":";

      for (const char *ReasonName : getReasonNames())
        SS << " " << ReasonName;

      return SS.str();
    }

  private:
    llvm::BasicBlock *BB;
    uint32_t Reasons;
  };

public:
  using BlockMap = std::map<MetaAddress, JumpTarget>;
  using RangesVector = std::vector<std::pair<MetaAddress, MetaAddress>>;
  using CSAAFactory = std::function<CPUStateAccessAnalysisPass *(void)>;

public:
  /// \param TheFunction the translated function.
  /// \param PCH ProgramCounterHandler instance.
  /// \param Binary reference to the information about a given binary, such as
  ///        segments and symbols.
  /// \param createCSAA a factory function able to create
  ///        CPUStateAccessAnalysisPass.
  JumpTargetManager(llvm::Function *TheFunction,
                    ProgramCounterHandler *PCH,
                    const BinaryFile &Binary,
                    CSAAFactory createCSAA);

  /// \brief Transform the IR to represent the request form of CFG
  void setCFGForm(CFGForm::Values NewForm,
                  MetaAddressSet *JumpTargetsWhitelist = nullptr);

  CFGForm::Values cfgForm() const { return CurrentCFGForm; }

  /// \brief Collect jump targets from the program's segments
  void harvestGlobalData();

  /// Handle a new program counter. We might already have a basic block for that
  /// program counter, or we could even have a translation for it. Return one
  /// of these, if appropriate.
  ///
  /// \param PC the new program counter.
  /// \param ShouldContinue an out parameter indicating whether the returned
  ///        basic block was just a placeholder or actually contains a
  ///        translation.
  ///
  /// \return the basic block to use from now on, or `nullptr` if the program
  ///         counter is not associated to a basic block.
  // TODO: return pair
  llvm::BasicBlock *newPC(MetaAddress PC, bool &ShouldContinue);

  /// \brief Save the PC-Instruction association for future use
  void registerInstruction(MetaAddress PC, llvm::Instruction *Instruction);

  /// \brief Return a pointer to the `exitTB` function
  ///
  /// `exitTB` is called when jump to the current value of the PC must be
  /// performed.
  llvm::Function *exitTB() { return ExitTB; }

  /// Clean OriginalInstructionAddresses and repopulate it coherently.
  void repopulate();

  /// \brief Pop from the list of program counters to explore
  ///
  /// \return a pair containing the PC and the initial block to use, or
  ///         JumpTarget::NoMoreTargets if we're done.
  BlockWithAddress peek();

  /// \brief Return true if no unexplored jump targets are available
  bool empty() { return Unexplored.empty(); }

  /// \brief Return true if the whole [\p Start,\p End) range is in an
  ///        executable segment
  bool isExecutableRange(MetaAddress Start, MetaAddress End) const {
    revng_assert(Start.isValid() and End.isValid());

    for (const std::pair<MetaAddress, MetaAddress> &Range : ExecutableRanges) {
      if (Range.first.addressLowerThanOrEqual(Start)
          and Start.addressLowerThan(Range.second)
          and Range.first.addressLowerThanOrEqual(End)
          and End.addressLowerThan(Range.second)) {
        return true;
      }
    }
    return false;
  }

  /// \brief Return true if the given PC can be executed by the current
  ///        architecture
  bool isPC(MetaAddress PC) const {
    revng_assert(PC.isValid());
    return isExecutableAddress(PC);
  }

  /// \brief Return true if the given PC is a jump target
  bool isJumpTarget(MetaAddress PC) const {
    revng_assert(PC.isValid());
    return JumpTargets.count(PC);
  }

  /// \brief Return true if the given basic block corresponds to a jump target
  bool isJumpTarget(llvm::BasicBlock *BB) {
    if (BB->empty())
      return false;

    MetaAddress PC = getPCFromNewPCCall(&*BB->begin());
    if (PC.isValid())
      return isJumpTarget(PC);

    return false;
  }

  /// \brief Return true if \p PC is in an executable segment
  bool isExecutableAddress(MetaAddress PC) const {
    revng_assert(PC.isValid());

    for (std::pair<MetaAddress, MetaAddress> Range : ExecutableRanges) {
      if (Range.first.addressLowerThanOrEqual(PC)
          and PC.addressLowerThan(Range.second)) {
        return true;
      }
    }
    return false;
  }

  /// \brief Get the basic block associated to the original address \p PC
  ///
  /// If the given address has never been met, assert.
  ///
  /// \param PC the PC for which a `BasicBlock` is requested.
  llvm::BasicBlock *getBlockAt(MetaAddress PC);

  /// \brief Return, and, if necessary, register the basic block associated to
  ///        \p PC
  ///
  /// This function can return `nullptr`.
  ///
  /// \param PC the PC for which a `BasicBlock` is requested.
  ///
  /// \return a `BasicBlock`, it might be newly created and empty, empty and
  ///         created in the past or even a `BasicBlock` already containing the
  ///         translated code.  It might also return `nullptr` if the PC is not
  ///         valid or another error occurred.
  llvm::BasicBlock *registerJT(MetaAddress PC, JTReason::Values Reason);

  bool hasJT(MetaAddress PC) {
    revng_assert(PC.isValid());
    return JumpTargets.count(PC) != 0;
  }

  BlockMap::const_iterator begin() const { return JumpTargets.begin(); }

  BlockMap::const_iterator end() const { return JumpTargets.end(); }

  void registerJT(llvm::BasicBlock *BB, JTReason::Values Reason) {
    registerJT(getBasicBlockPC(notNull(BB)), Reason);
  }

  // TODO: this is a likely approach is broken, it depends on the order
  /// \brief As registerJT, but only if the JT has already been registered
  void markJT(MetaAddress PC, JTReason::Values Reason) {
    revng_assert(PC.isValid());

    if (isJumpTarget(PC))
      registerJT(PC, Reason);
  }

  /// \brief Checks if \p BB is a basic block generated during translation
  bool isTranslatedBB(llvm::BasicBlock *BB) const {
    return BB != anyPC() && BB != unexpectedPC() && BB != dispatcher()
           && BB != dispatcherFail();
  }

  /// \brief Return the dispatcher basic block.
  ///
  /// \note Do not use this for comparison with successors of translated code,
  ///       use isTranslatedBB instead.
  llvm::BasicBlock *dispatcher() const { return Dispatcher; }

  /// \brief Return the basic block handling an unknown PC in the dispatcher
  llvm::BasicBlock *dispatcherFail() const { return DispatcherFail; }

  /// \brief Return the basic block handling a jump to any PC
  llvm::BasicBlock *anyPC() const { return AnyPC; }

  /// \brief Return the basic block handling a jump to an unexpected PC
  llvm::BasicBlock *unexpectedPC() const { return UnexpectedPC; }

  // TODO: can this be replaced by the corresponding method in
  // GeneratedCodeBasicInfo?
  /// \brief Get the PC associated and the size of the original instruction
  std::pair<MetaAddress, uint64_t>
  getPC(llvm::Instruction *TheInstruction) const;

  // TODO: can this be replaced by the corresponding method in
  // GeneratedCodeBasicInfo?
  MetaAddress getNextPC(llvm::Instruction *TheInstruction) const {
    auto Pair = getPC(TheInstruction);
    return Pair.first + Pair.second;
  }

  MaterializedValue
  readFromPointer(llvm::Constant *Pointer, BinaryFile::Endianess E);

  /// \brief Increment the counter of emitted branches since the last reset
  void recordNewBranches(llvm::BasicBlock *Source, size_t Count) {
    AVIPCWhiteList.insert(getPC(Source->getTerminator()).first);
    NewBranches += Count;
  }

  /// \brief Finalizes information about the jump targets
  ///
  /// Call this function once no more jump targets can be discovered.  It will
  /// fix all the pending information. In particular, those pointers to code
  /// that have never been touched will be considered and their pointee will be
  /// marked with UnusedGlobalData.
  ///
  /// This function also fixes the "anypc" and "unexpectedpc" basic blocks to
  /// their proper behavior.
  void finalizeJumpTargets() {
    fixPostHelperPC();

    translateIndirectJumps();

    unsigned ReadSize = Binary.architecture().pointerSize() / 8;
    for (MetaAddress MemoryAddress : UnusedCodePointers) {
      // Read using the original endianess, we want the correct address
      uint64_t RawPC = *Binary.readRawValue(MemoryAddress, ReadSize);
      auto PC = fromPC(RawPC);

      // Set as reason UnusedGlobalData and ensure it's not empty
      llvm::BasicBlock *BB = registerJT(PC, JTReason::UnusedGlobalData);

      // TODO: can this happen?
      revng_assert(BB != nullptr);

      revng_assert(!BB->empty());
    }

    // We no longer need this information
    freeContainer(UnusedCodePointers);
  }

  MetaAddress fromPC(uint64_t PC) const { return Binary.fromPC(PC); }
  MetaAddress fromAbsolute(uint64_t Address) const {
    return Binary.fromGeneric(Address);
  }

  MetaAddress fromPCStore(llvm::StoreInst *Store) {
    auto *Constant = llvm::cast<llvm::ConstantInt>(Store->getValueOperand());
    return fromPC(Constant->getLimitedValue());
  }

  void createJTReasonMD() {
    using namespace llvm;

    Function *CallMarker = TheModule.getFunction("function_call");
    if (CallMarker != nullptr) {
      auto unwrapBA = [](Value *V) {
        return cast<BlockAddress>(V)->getBasicBlock();
      };
      for (User *U : CallMarker->users()) {
        if (CallInst *Call = dyn_cast<CallInst>(U)) {
          if (isa<BlockAddress>(Call->getOperand(0)))
            registerJT(unwrapBA(Call->getOperand(0)), JTReason::Callee);
          registerJT(unwrapBA(Call->getOperand(1)), JTReason::ReturnAddress);
        }
      }
    }

    // Tag each jump target with its reasons
    for (auto &P : JumpTargets) {
      JumpTarget &JT = P.second;
      Instruction *T = JT.head()->getTerminator();
      revng_assert(T != nullptr);

      std::vector<Metadata *> Reasons;
      for (const char *ReasonName : JT.getReasonNames())
        Reasons.push_back(MDString::get(Context, ReasonName));

      T->setMetadata("revng.jt.reasons", MDTuple::get(Context, Reasons));
    }
  }

  unsigned delaySlotSize() const {
    return Binary.architecture().delaySlotSize();
  }

  const BinaryFile &binary() const { return Binary; }

  void registerReadRange(MetaAddress Address, uint64_t Size);

  const interval_set &readRange() const { return ReadIntervalSet; }

  std::string nameForAddress(MetaAddress Address, uint64_t Size = 1) const {
    return Binary.nameForAddress(Address, Size);
  }

  /// \brief Register a simple literal collected during translation for
  ///        harvesting
  ///
  /// A simple literal is a literal value found in the input program that is
  /// simple enough not to require more sophisticated analyses. The typcal
  /// example is the return address of a function call, that is provided to use
  /// by libtinycode in full.
  ///
  /// Simple literals are registered as possible jump targets before attempting
  /// more expensive techniques.
  void registerSimpleLiteral(MetaAddress Address) {
    SimpleLiterals.insert(Address);
  }

  ProgramCounterHandler *programCounterHandler() { return PCH; }

private:
  void fixPostHelperPC();

  std::set<llvm::BasicBlock *> computeUnreachable() const;

  void assertNoUnreachable() const;

  /// \brief Translate the non-constant jumps into jumps to the dispatcher
  void translateIndirectJumps();

  /// \brief Helper function to check if an instruction is a call to `newpc`
  ///
  /// \return 0 if \p I is not a call to `newpc`, otherwise the PC address of
  ///         associated to the call to `newpc`
  MetaAddress getPCFromNewPCCall(const llvm::Instruction *I) {
    if (auto *CallNewPC = llvm::dyn_cast<llvm::CallInst>(I)) {
      if (CallNewPC->getCalledFunction() == nullptr
          || CallNewPC->getCalledFunction()->getName() != "newpc")
        return MetaAddress::invalid();

      return MetaAddress::fromConstant(CallNewPC->getArgOperand(0));
    }

    return MetaAddress::invalid();
  }

  /// \brief Erase \p I, and deregister it in case it's a call to `newpc`
  void eraseInstruction(llvm::Instruction *I) {
    revng_assert(I->use_empty());

    MetaAddress PC = getPCFromNewPCCall(I);
    if (PC.isValid())
      OriginalInstructionAddresses.erase(PC);
    I->eraseFromParent();
  }

  /// \brief Drop \p Start and all the descendants, stopping when a JT is met
  void purgeTranslation(llvm::BasicBlock *Start);

  /// \brief Check if \p BB has at least a predecessor, excluding the dispatcher
  bool hasPredecessors(llvm::BasicBlock *BB) const;

  /// \brief Rebuild the dispatcher switch
  ///
  /// Depending on the CFG form we're currently adopting the dispatcher might go
  /// to all the jump targets or only to those who have no other predecessor.
  void rebuildDispatcher(MetaAddressSet *Whitelist);

  void prepareDispatcher();

  template<typename value_type, unsigned endian>
  void findCodePointers(MetaAddress StartVirtualAddress,
                        const unsigned char *Start,
                        const unsigned char *End);

  void harvestWithAVI();

  void harvest();

  /// \brief Decorate memory accesses with information about CSV aliasing
  void aliasAnalysis();

  MetaAddressSet inflateAVIWhitelist();

private:
  using InstructionMap = std::map<MetaAddress, llvm::Instruction *>;

  llvm::Module &TheModule;
  llvm::LLVMContext &Context;
  llvm::Function *TheFunction;
  /// Holds the association between a PC and the last generated instruction for
  /// the previous instruction.
  InstructionMap OriginalInstructionAddresses;
  /// Holds the association between a PC and a BasicBlock.
  BlockMap JumpTargets;
  /// Queue of program counters we still have to translate.
  std::vector<BlockWithAddress> Unexplored;

  llvm::Function *ExitTB;
  RangesVector ExecutableRanges;

  llvm::BasicBlock *Dispatcher;
  llvm::SwitchInst *DispatcherSwitch;
  llvm::BasicBlock *DispatcherFail;
  llvm::BasicBlock *AnyPC;
  llvm::BasicBlock *UnexpectedPC;

  const BinaryFile &Binary;

  unsigned NewBranches = 0;

  std::set<MetaAddress> UnusedCodePointers;
  interval_set ReadIntervalSet;

  CFGForm::Values CurrentCFGForm;
  std::set<llvm::BasicBlock *> ToPurge;
  std::set<MetaAddress> SimpleLiterals;
  CSAAFactory createCSAA;

  ProgramCounterHandler *PCH;

  MetaAddressSet AVIPCWhiteList;
};

template<>
struct BlackListTrait<const JumpTargetManager &, llvm::BasicBlock *>
  : BlackListTraitBase<const JumpTargetManager &> {
  using BlackListTraitBase<const JumpTargetManager &>::BlackListTraitBase;
  bool isBlacklisted(llvm::BasicBlock *Value) {
    return !this->Obj.isTranslatedBB(Value);
  }
};

inline BlackListTrait<const JumpTargetManager &, llvm::BasicBlock *>
make_blacklist(const JumpTargetManager &JTM) {
  return BlackListTrait<const JumpTargetManager &, llvm::BasicBlock *>(JTM);
}
