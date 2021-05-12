#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <map>
#include <vector>

#include "llvm/ADT/SmallSet.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Pass.h"
#include "llvm/Support/ErrorOr.h"

#include "revng/Support/ProgramCounterHandler.h"
#include "revng/Support/revng.h"

#include "JumpTargetManager.h"
#include "PTCDump.h"

// Forward declarations
namespace llvm {
class BasicBlock;
class CallInst;
class Function;
class MDNode;
class Module;
} // namespace llvm

class JumpTargetManager;
class VariableManager;

/// \brief Expands a PTC instruction to LLVM IR
class InstructionTranslator {
public:
  using LabeledBlocksMap = std::map<std::string, llvm::BasicBlock *>;

  /// \param Builder the IRBuilder to be used to create the translated
  ///                code.
  /// \param Variables reference to the VariableManager.
  /// \param JumpTargets reference to the JumpTargetManager.
  /// \param Blocks reference to a `vector` of `BasicBlock`s used to keep track
  ///        on which `BasicBlock`s the InstructionTranslator worked on, for
  ///        further processing.
  /// \param SourceArchitecture the input architecture.
  /// \param TargetArchitecture the output architecture.
  InstructionTranslator(llvm::IRBuilder<> &Builder,
                        VariableManager &Variables,
                        JumpTargetManager &JumpTargets,
                        std::vector<llvm::BasicBlock *> Blocks,
                        const Architecture &SourceArchitecture,
                        const Architecture &TargetArchitecture,
                        ProgramCounterHandler *PCH);

  /// \brief Result status of the translation of a PTC opcode
  enum TranslationResult {
    Abort, ///< An error occurred during translation, call abort and stop
    Stop, ///< Do not proceed with translation
    Success ///< The translation was successful
  };

  /// \brief Handle a new instruction from the input code
  ///
  /// \param Instr the newly met PTCInstruction;
  /// \param Next the PTCInstruction immediately following \p Instr,
  ///             or `nullptr` if \p Instr is the last instruction
  ///             translated by libtinycode.
  /// \param EndPC PC of the instruction coming after the next
  ///              instruction after the last translated by
  ///              libtinycode.
  /// \param IsFirst true, if \p Instr is the first instruction
  ///                translated by libtinycode.
  ///
  /// \return a tuple with 4 entries: the
  ///         InstructionTranslator::TranslationResult, an `MDNode` containing
  ///         the disassembled instruction and the value of the PC and two
  ///         `MetaAddress` representing the current and next PC.
  // TODO: rename to newPC
  // TODO: the signature of this function is ugly
  std::tuple<TranslationResult, llvm::MDNode *, MetaAddress, MetaAddress>
  newInstruction(PTCInstruction *Instr,
                 PTCInstruction *Next,
                 MetaAddress StartPC,
                 MetaAddress EndPC,
                 bool IsFirst,
                 MetaAddress AbortAt);

  /// \brief Translate an ordinary instruction
  ///
  /// \param Instr the instruction to translate.
  /// \param PC the PC associated to \p Instr.
  /// \param NextPC the PC associated to instruction after \p Instr.
  ///
  /// \return see InstructionTranslator::TranslationResult.
  TranslationResult
  translate(PTCInstruction *Instr, MetaAddress PC, MetaAddress NextPC);

  /// \brief Translate a call to an helper
  ///
  /// \param Instr the PTCInstruction of the call to the helper.
  ///
  /// \return see InstructionTranslator::TranslationResult.
  TranslationResult translateCall(PTCInstruction *Instr);

  /// \brief Handle calls to `newPC` marker and emit coverage information
  ///
  /// \param CoveragePath path where the coverage information should be stored.
  void finalizeNewPCMarkers(std::string &CoveragePath);

  /// \brief Notifies InstructionTranslator about a new PTC translation
  void reset() { LabeledBasicBlocks.clear(); }

  /// \brief Preprocess the translated instructions
  ///
  /// Check if the translated code contains a delay slot and return a blacklist
  /// of the PTC_INSTRUCTION_op_debug_insn_start instructions that have to be
  /// ignored to merge the delay slot into the branch instruction.
  llvm::SmallSet<unsigned, 1> preprocess(PTCInstructionList *Instructions);

  void registerDirectJumps();

private:
  llvm::ErrorOr<std::vector<llvm::Value *>>
  translateOpcode(PTCOpcode Opcode,
                  std::vector<uint64_t> ConstArguments,
                  std::vector<llvm::Value *> InArguments);

private:
  llvm::IRBuilder<> &Builder;
  VariableManager &Variables;
  JumpTargetManager &JumpTargets;
  std::map<std::string, llvm::BasicBlock *> LabeledBasicBlocks;
  std::vector<llvm::BasicBlock *> Blocks;
  llvm::Module &TheModule;

  llvm::Function *TheFunction;

  const Architecture &SourceArchitecture;
  const Architecture &TargetArchitecture;

  llvm::Function *NewPCMarker;

  MetaAddress LastPC;
  llvm::Type *MetaAddressStruct;

  ProgramCounterHandler *PCH;
  llvm::SmallVector<llvm::BasicBlock *, 4> ExitBlocks;
};
