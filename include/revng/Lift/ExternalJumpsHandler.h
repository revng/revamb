#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <vector>

#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instruction.h"
#include "llvm/Pass.h"

#include "revng/Support/revng.h"

#include "BinaryFile.h"

class ProgramCounterHandler;

/// \brief Inject code to support jumping in non-translated code and handling
///        the comeback.
///
/// This pass changes the default case of the dispatcher by checking if you're
/// trying to jump to an address that is not in one of the executable segments.
/// If so, the relevant a setjmp is performed, the CPU state is serialized to
/// the actual phsyical registers and then a jump to the target address is
/// performed. At this point the jump might be successful or it might fail.
///
/// Symmetrically, with the help of support.c, a signal handler is installed
/// which detects segmentation faults and checks if an attempt to jump to an
/// executable segment was performed.
/// If this is the case, we perform a longjmp to get back into the proper
/// context, we restore the relevant registers from the data structures provided
/// by the signal handler and then jump to the dispatcher to resume execution.
class ExternalJumpsHandler {
private:
  llvm::LLVMContext &Context;
  QuickMetadata QMD;
  llvm::Module &TheModule;
  llvm::Function &TheFunction;

  BinaryFile &TheBinary;
  const Architecture &Arch;
  llvm::BasicBlock *Dispatcher;
  ProgramCounterHandler *PCH;

public:
  /// \param TheFunction the root function.
  ExternalJumpsHandler(BinaryFile &TheBinary,
                       llvm::BasicBlock *Dispatcher,
                       llvm::Function &TheFunction,
                       ProgramCounterHandler *PCH);

public:
  /// \brief Creates the jump out and jump back in handling infrastructure.
  void createExternalJumpsHandler();

private:
  /// \brief Create the basic blocks to handle jumping to external code.
  ///
  /// Prepare serialize_and_jump_out, which writes in physical registers the
  /// values of all ABI-related CSVs and then blindly jumps to the content of
  /// the program counter CSV.
  llvm::BasicBlock *createSerializeAndJumpOut();

  /// \brief Create the setjump basic block.
  ///
  /// This basic block will perform a setjmp to save the context (the stack
  /// pointer in particular) and then go to serialize_and_jump_out.
  /// The second return of setjmp instead will deserialize the CPU state and go
  /// back to the dispatcher.
  llvm::BasicBlock *
  createSetjmp(llvm::BasicBlock *FirstReturn, llvm::BasicBlock *SecondReturn);

  /// \brief Prepare a list of the executable segments that can be easily
  ///        consumed by support.c.
  ///
  /// This method creates three global variables:
  ///
  /// * an unamed array of uint64_t large as twice the number of executable
  ///   segments, where the even entries contain the start address of a segment
  ///   and odd ones the end address.
  /// * "segment_boundaries": a `uint64_t *` targeting the previous array.
  /// * "segments_count": an uint64_t containing the number of executable
  ///   segments.
  void buildExecutableSegmentsList();

  /// \brief Creates the basic block taking care of deserializing the CPU state
  ///        to the CSVs.
  ///
  /// The CPU state is restored from the mcontext_t field in the struct provided
  /// by the kernel to the signal handler.
  llvm::BasicBlock *createReturnFromExternal();
};
