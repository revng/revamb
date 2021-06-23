#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <iostream>

#include "revng/Support/MetaAddress.h"

#include "ptc.h"

/// Write to a stream the string representation of the PTC instruction with the
/// specified index within the instruction list.
///
/// \param Result the output stream.
/// \param Instructions the instruction list.
/// \param Index the index of the target instruction in the instruction list.
///
/// \return EXIT_SUCCESS in case of success, EXIT_FAILURE otherwise.
int dumpInstruction(std::ostream &Result,
                    PTCInstructionList *Instructions,
                    unsigned Index);

/// Write to a stream all the instructions in an instruction list.
///
/// \param Result the output stream.
/// \param Instructions the instruction list.
///
/// \return EXIT_SUCCESS in case of success, EXIT_FAILURE otherwise.
int dumpTranslation(MetaAddress VirtualAddress,
                    std::ostream &Result,
                    PTCInstructionList *Instructions);

/// Write to a stream the dissasembled version of the instruction at the
/// specified program counter.
///
/// \param Result the output stream
/// \param PC the program counter in the current context.
/// \param MaxSize the maximum number of bytes to disassemble.
/// \param InstructionCount the maximum number of instructions to disassemble.
void disassemble(std::ostream &Result,
                 MetaAddress PC,
                 uint32_t MaxBytes = 4096,
                 uint32_t InstructionCount = 4096);
