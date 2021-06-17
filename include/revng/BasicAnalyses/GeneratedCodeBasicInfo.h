#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <map>
#include <utility>

#include "llvm/IR/Dominators.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"
#include "llvm/Support/Casting.h"

#include "revng/Support/BlockType.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/ProgramCounterHandler.h"
#include "revng/Support/revng.h"

// Forward declarations
namespace llvm {
class BasicBlock;
class GlobalVariable;
class Instruction;
class MDNode;
} // namespace llvm

static const char *JTReasonMDName = "revng.jt.reasons";

/// \brief Pass to collect basic information about the generated code
///
/// This pass provides useful information for other passes by extracting them
/// from the generated IR, and possibly caching them.
///
/// It provides details about the input architecture such as the size of its
/// delay slot, the name of the program counter register and so on. It also
/// provides information about the generated basic blocks, distinguishing
/// between basic blocks generated due to translation and dispatcher-related
/// basic blocks.
class GeneratedCodeBasicInfo {
public:
  GeneratedCodeBasicInfo() :
    ArchType(llvm::Triple::ArchType::UnknownArch),
    InstructionAlignment(0),
    DelaySlotSize(0),
    PC(nullptr),
    Dispatcher(nullptr),
    DispatcherFail(nullptr),
    AnyPC(nullptr),
    UnexpectedPC(nullptr),
    PCRegSize(0),
    RootFunction(nullptr),
    MetaAddressStruct(nullptr),
    PCH() {}

  void run(llvm::Module &M);

  /// \brief Handle the invalidation of this information.
  bool invalidate(llvm::Module &,
                  const llvm::PreservedAnalyses &,
                  llvm::ModuleAnalysisManager::Invalidator &) {
    return false;
  }

  bool invalidate(llvm::Function &,
                  const llvm::PreservedAnalyses &,
                  llvm::FunctionAnalysisManager::Invalidator &) {
    return false;
  }

  /// \brief Return the type of basic block, see BlockType.
  static BlockType::Values getType(llvm::BasicBlock *BB) {
    return getType(BB->getTerminator());
  }

  /// \brief Return the type of basic block, see BlockType.
  static bool isPartOfRootDispatcher(llvm::BasicBlock *BB) {
    auto Type = getType(BB->getTerminator());
    return (Type == BlockType::RootDispatcherBlock
            or Type == BlockType::RootDispatcherHelperBlock);
  }

  static BlockType::Values getType(llvm::Instruction *T) {
    using namespace llvm;

    revng_assert(T != nullptr);
    revng_assert(T->isTerminator());
    MDNode *MD = T->getMetadata(BlockTypeMDName);

    BasicBlock *BB = T->getParent();
    if (BB == &BB->getParent()->getEntryBlock())
      return BlockType::EntryPoint;

    if (MD == nullptr) {
      Instruction *First = &*T->getParent()->begin();
      if (CallInst *Call = getCallTo(First, "newpc"))
        if (getLimitedValue(Call->getArgOperand(2)) == 1)
          return BlockType::JumpTargetBlock;

      return BlockType::TranslatedBlock;
    }

    auto *BlockTypeMD = cast<MDTuple>(MD);

    QuickMetadata QMD(getContext(T));
    return BlockType::fromName(QMD.extract<llvm::StringRef>(BlockTypeMD, 0));
  }

  uint32_t getJTReasons(llvm::BasicBlock *BB) const {
    return getJTReasons(BB->getTerminator());
  }

  uint32_t getJTReasons(llvm::Instruction *T) const {
    using namespace llvm;

    revng_assert(T->isTerminator());

    uint32_t Result = 0;

    MDNode *Node = T->getMetadata(JTReasonMDName);
    auto *Tuple = cast_or_null<MDTuple>(Node);
    revng_assert(Tuple != nullptr);

    for (Metadata *ReasonMD : Tuple->operands()) {
      StringRef Text = cast<MDString>(ReasonMD)->getString();
      Result |= static_cast<uint32_t>(JTReason::fromName(Text));
    }

    return Result;
  }

  KillReason::Values getKillReason(llvm::BasicBlock *BB) const {
    return getKillReason(BB->getTerminator());
  }

  KillReason::Values getKillReason(llvm::Instruction *T) const {
    using namespace llvm;

    revng_assert(T->isTerminator());

    auto *NoReturnMD = T->getMetadata("noreturn");
    if (auto *NoreturnTuple = dyn_cast_or_null<MDTuple>(NoReturnMD)) {
      QuickMetadata QMD(getContext(T));
      return KillReason::fromName(QMD.extract<StringRef>(NoreturnTuple, 0));
    }

    return KillReason::NonKiller;
  }

  bool isKiller(llvm::BasicBlock *BB) const {
    return isKiller(BB->getTerminator());
  }

  bool isKiller(llvm::Instruction *T) const {
    revng_assert(T->isTerminator());
    return getKillReason(T) != KillReason::NonKiller;
  }

  /// \brief Return the value to which instructions must be aligned in the input
  ///        architecture
  unsigned instructionAlignment() const { return InstructionAlignment; }

  /// \brief Return the size of the delay slot for the input architecture
  unsigned delaySlotSize() const { return DelaySlotSize; }

  /// \brief Return the CSV representing the stack pointer
  llvm::GlobalVariable *spReg() const { return SP; }

  llvm::Triple::ArchType arch() const { return ArchType; }

  /// \brief Check if \p GV is the stack pointer CSV
  bool isSPReg(const llvm::GlobalVariable *GV) const {
    revng_assert(SP != nullptr);
    return GV == SP;
  }

  bool isSPReg(const llvm::Value *V) const {
    if (auto *GV = llvm::dyn_cast<const llvm::GlobalVariable>(V))
      return isSPReg(GV);
    return false;
  }

  // TODO: this method should probably be deprecated
  /// \brief Return the CSV representing the program counter
  llvm::GlobalVariable *pcReg() const { return PC; }

  // TODO: this method should probably be deprecated
  unsigned pcRegSize() const { return PCRegSize; }

  // TODO: this method should probably be deprecated
  /// \brief Check if \p GV is the program counter CSV
  bool isPCReg(const llvm::GlobalVariable *GV) const {
    revng_assert(PC != nullptr);
    return GV == PC;
  }

  // TODO: this method should probably be deprecated
  bool isServiceRegister(const llvm::Value *V) const {
    auto *GV = llvm::dyn_cast<llvm::GlobalVariable>(V);
    return GV != nullptr and (isPCReg(GV) or isSPReg(GV));
  }

  const ProgramCounterHandler *programCounterHandler() {
    if (not PCH) {
      llvm::Module *M = RootFunction->getParent();
      PCH = ProgramCounterHandler::fromModule(ArchType, M);
    }

    return PCH.get();
  }

  template<typename T>
  ProgramCounterHandler::DispatcherInfo
  buildDispatcher(T &Targets,
                  llvm::IRBuilder<> &Builder,
                  llvm::BasicBlock *Default = nullptr) {
    ProgramCounterHandler::DispatcherTargets TargetsPairs;
    TargetsPairs.reserve(Targets.size());
    for (MetaAddress MA : Targets)
      TargetsPairs.push_back({ MA, getBlockAt(MA) });

    if (Default == nullptr)
      Default = UnexpectedPC;

    auto IBDHB = BlockType::IndirectBranchDispatcherHelperBlock;
    return programCounterHandler()->buildDispatcher(TargetsPairs,
                                                    Builder,
                                                    Default,
                                                    { IBDHB });
  }

  /// \brief Return the basic block associated to \p PC
  ///
  /// Returns nullptr if the PC doesn't have a basic block (yet)
  llvm::BasicBlock *getBlockAt(MetaAddress PC) const {
    auto It = JumpTargets.find(PC);
    if (It == JumpTargets.end())
      return nullptr;

    return It->second;
  }

  /// \brief Return true if the basic block is a jump target
  static bool isJumpTarget(llvm::BasicBlock *BB) {
    return getType(BB->getTerminator()) == BlockType::JumpTargetBlock;
  }

  llvm::BasicBlock *getJumpTargetBlock(llvm::BasicBlock *BB);

  MetaAddress getJumpTarget(llvm::BasicBlock *BB) {
    return getPCFromNewPC(getJumpTargetBlock(BB));
  }

  bool isJump(llvm::BasicBlock *BB) const {
    return isJump(BB->getTerminator());
  }

  /// \brief Return true if \p T represents a jump in the input assembly
  ///
  /// Return true if \p T targets include only dispatcher-related basic blocks
  /// and jump targets.
  bool isJump(llvm::Instruction *T) const {
    revng_assert(T != nullptr);
    revng_assert(T->isTerminator());

    for (llvm::BasicBlock *Successor : successors(T)) {
      if (not(Successor->empty() or Successor == Dispatcher
              or Successor == DispatcherFail or Successor == AnyPC
              or Successor == UnexpectedPC or isJumpTarget(Successor)))
        return false;
    }

    return true;
  }

  /// \brief Return true if \p BB is the result of translating some code
  ///
  /// Return false if \p BB is a dispatcher-related basic block.
  static bool isTranslated(llvm::BasicBlock *BB) {
    BlockType::Values Type = getType(BB);
    return (Type == BlockType::TranslatedBlock
            or Type == BlockType::JumpTargetBlock);
  }

  /// \brief Return the program counter of the next (i.e., fallthrough)
  ///        instruction of \p TheInstruction
  MetaAddress getNextPC(llvm::Instruction *TheInstruction) const {
    auto Pair = getPC(TheInstruction);
    return Pair.first + Pair.second;
  }

  llvm::BasicBlock *getCallReturnBlock(llvm::BasicBlock *BB) const {
    using namespace llvm;
    CallInst *FunctionCallMarker = getFunctionCall(BB);
    revng_assert(FunctionCallMarker != nullptr);
    auto *FallthroughBA = cast<BlockAddress>(FunctionCallMarker->getOperand(1));
    return FallthroughBA->getBasicBlock();
  }

  auto getBlocksGeneratedByPC(MetaAddress PC) {
    // Lazily initialize the pc-to-BasicBlock cache
    if (PCToBlockCache.size() == 0)
      initializePCToBlockCache();

    auto GetSecond = [](PCToBlockMap::value_type &Element) {
      return Element.second;
    };

    auto [Start, End] = PCToBlockCache.equal_range(PC);
    return llvm::make_range(llvm::map_iterator(Start, GetSecond),
                            llvm::map_iterator(End, GetSecond));
  }

  llvm::BasicBlock *anyPC() const { return AnyPC; }

  llvm::BasicBlock *unexpectedPC() const { return UnexpectedPC; }

  llvm::BasicBlock *dispatcher() const { return Dispatcher; }

  const llvm::ArrayRef<llvm::GlobalVariable *> csvs() const { return CSVs; }

  class CSVsUsage {
  public:
    void sort() {
      std::sort(Read.begin(), Read.end());
      std::sort(Written.begin(), Written.end());
    }

  public:
    std::vector<llvm::GlobalVariable *> Read;
    std::vector<llvm::GlobalVariable *> Written;
  };

  static CSVsUsage getCSVUsedByHelperCall(llvm::Instruction *Call) {
    return *getCSVUsedByHelperCallIfAvailable(Call);
  }

  static llvm::Optional<CSVsUsage>
  getCSVUsedByHelperCallIfAvailable(llvm::Instruction *Call) {
    revng_assert(isCallToHelper(Call));

    const llvm::Module *M = getModule(Call);
    const auto LoadMDKind = M->getMDKindID("revng.csvaccess.offsets.load");
    const auto StoreMDKind = M->getMDKindID("revng.csvaccess.offsets.store");

    if (Call->getMetadata(LoadMDKind) == nullptr
        and Call->getMetadata(StoreMDKind) == nullptr) {
      return {};
    }

    CSVsUsage Result;
    Result.Read = extractCSVs(Call, LoadMDKind);
    Result.Written = extractCSVs(Call, StoreMDKind);
    return Result;
  }

  const std::vector<llvm::GlobalVariable *> &abiRegisters() const {
    return ABIRegisters;
  }

  bool isABIRegister(llvm::GlobalVariable *CSV) const {
    return ABIRegistersSet.count(CSV) != 0;
  }
  llvm::Constant *toConstant(const MetaAddress &Address) {
    revng_assert(MetaAddressStruct != nullptr);
    return Address.toConstant(MetaAddressStruct);
  }

  MetaAddress fromPC(uint64_t PC) const {
    return MetaAddress::fromPC(ArchType, PC);
  }

  struct SuccessorsList {
    bool AnyPC = false;
    bool UnexpectedPC = false;
    bool Other = false;
    std::set<MetaAddress> Addresses;

    static SuccessorsList other() {
      SuccessorsList Result;
      Result.Other = true;
      return Result;
    }

    bool hasSuccessors() const {
      return AnyPC or UnexpectedPC or Other or Addresses.size() != 0;
    }

    void dump() const debug_function { dump(dbg); }

    template<typename O>
    void dump(O &Output) const {
      Output << "AnyPC: " << AnyPC << "\n";
      Output << "UnexpectedPC: " << UnexpectedPC << "\n";
      Output << "Other: " << Other << "\n";
      Output << "Addresses:\n";
      for (const MetaAddress &Address : Addresses) {
        Output << "  ";
        Address.dump(Output);
        Output << "\n";
      }
    }
  };

  SuccessorsList getSuccessors(llvm::BasicBlock *BB) const;

  llvm::Function *root() const { return RootFunction; }

  llvm::SmallVector<std::pair<llvm::BasicBlock *, bool>, 4>
  blocksByPCRange(MetaAddress Start, MetaAddress End);

  static MetaAddress getPCFromNewPC(llvm::Instruction *I) {
    if (llvm::CallInst *NewPCCall = getCallTo(I, "newpc")) {
      return MetaAddress::fromConstant(NewPCCall->getArgOperand(0));
    } else {
      return MetaAddress::invalid();
    }
  }

  static MetaAddress getPCFromNewPC(llvm::BasicBlock *BB) {
    return getPCFromNewPC(&*BB->begin());
  }

private:
  void initializePCToBlockCache();

private:
  static std::vector<llvm::GlobalVariable *>
  extractCSVs(llvm::Instruction *Call, unsigned MDKindID) {
    using namespace llvm;

    std::vector<GlobalVariable *> Result;
    auto *Tuple = cast_or_null<MDTuple>(Call->getMetadata(MDKindID));
    if (Tuple == nullptr)
      return Result;

    QuickMetadata QMD(getContext(Call));

    auto OperandsRange = QMD.extract<MDTuple *>(Tuple, 1)->operands();
    for (const MDOperand &Operand : OperandsRange) {
      auto *CSV = QMD.extract<Constant *>(Operand.get());
      Result.push_back(cast<GlobalVariable>(CSV));
    }

    return Result;
  }

  const llvm::DominatorTree &getDomTree(llvm::Function *F) {
    auto It = DTMap.find(F);
    if (It == DTMap.end()) {
      llvm::DominatorTree &Result = DTMap[F];
      Result.recalculate(*F);
      return Result;
    }
    return It->second;
  }

private:
  llvm::Triple::ArchType ArchType;
  uint32_t InstructionAlignment;
  uint32_t DelaySlotSize;
  llvm::GlobalVariable *PC;
  llvm::GlobalVariable *SP;
  llvm::BasicBlock *Dispatcher;
  llvm::BasicBlock *DispatcherFail;
  llvm::BasicBlock *AnyPC;
  llvm::BasicBlock *UnexpectedPC;
  std::map<MetaAddress, llvm::BasicBlock *> JumpTargets;
  unsigned PCRegSize;
  llvm::Function *RootFunction;
  std::vector<llvm::GlobalVariable *> CSVs;
  std::vector<llvm::GlobalVariable *> ABIRegisters;
  std::set<llvm::GlobalVariable *> ABIRegistersSet;
  llvm::StructType *MetaAddressStruct;
  llvm::Function *NewPC;
  std::unique_ptr<ProgramCounterHandler> PCH;
  using PCToBlockMap = std::multimap<MetaAddress, llvm::BasicBlock *>;
  PCToBlockMap PCToBlockCache;
  std::map<llvm::Function *, llvm::DominatorTree> DTMap;
};

template<>
struct BlackListTrait<const GeneratedCodeBasicInfo &, llvm::BasicBlock *>
  : BlackListTraitBase<const GeneratedCodeBasicInfo &> {
  using BlackListTraitBase<const GeneratedCodeBasicInfo &>::BlackListTraitBase;
  bool isBlacklisted(llvm::BasicBlock *Value) const {
    return !this->Obj.isTranslated(Value);
  }
};

/// An analysis pass that computes a \c GCBI result. The result of
/// this analysis is invalidated each time the analysis is called.
class GeneratedCodeBasicInfoAnalysis
  : public llvm::AnalysisInfoMixin<GeneratedCodeBasicInfoAnalysis> {
  friend llvm::AnalysisInfoMixin<GeneratedCodeBasicInfoAnalysis>;
  static llvm::AnalysisKey Key;

public:
  using Result = GeneratedCodeBasicInfo;
  /// \note If a MPM is used, then make sure to register the
  /// analysis manually and use a proxy.
  Result run(llvm::Module &M, llvm::ModuleAnalysisManager &);
  Result run(llvm::Function &F, llvm::FunctionAnalysisManager &);
};

/// Legacy pass manager pass to access GCBI.
class GeneratedCodeBasicInfoWrapperPass : public llvm::ModulePass {
  std::unique_ptr<GeneratedCodeBasicInfo> GCBI;

public:
  static char ID;

  GeneratedCodeBasicInfoWrapperPass() : llvm::ModulePass(ID) {}

  GeneratedCodeBasicInfo &getGCBI() { return *GCBI; }

  bool runOnModule(llvm::Module &M) override;
  void releaseMemory() override;
  void getAnalysisUsage(llvm::AnalysisUsage &AU) const override {
    AU.setPreservesAll();
  }
};
