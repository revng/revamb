/// \file PTCDump.cpp
/// \brief This file handles dumping PTC to text

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#include "revng/Lift/PTCDump.h"
#include "revng/Lift/PTCInterface.h"
#include "revng/Support/Assert.h"

#include "PTCDump.h"
#include "PTCInterface.h"

static const int MAX_TEMP_NAME_LENGTH = 128;

static void getTemporaryName(char *Buffer,
                             size_t BufferSize,
                             PTCInstructionList *Instructions,
                             unsigned TemporaryId) {
  PTCTemp *Temporary = ptc_temp_get(Instructions, TemporaryId);

  if (ptc_temp_is_global(Instructions, TemporaryId))
    strncpy(Buffer, Temporary->name, BufferSize);
  else if (Temporary->temp_local)
    snprintf(Buffer,
             BufferSize,
             "loc%u",
             TemporaryId - Instructions->global_temps);
  else
    snprintf(Buffer,
             BufferSize,
             "tmp%u",
             TemporaryId - Instructions->global_temps);
}

int dumpInstruction(std::ostream &Result,
                    PTCInstructionList *Instructions,
                    unsigned Index) {
  size_t i = 0;
  // TODO: this should stay in Architecture
  int is64 = 0;
  PTCInstruction &Instruction = Instructions->instructions[Index];

  PTCOpcode Opcode = Instruction.opc;
  PTCOpcodeDef *Definition = ptc_instruction_opcode_def(&ptc, &Instruction);
  char TemporaryName[MAX_TEMP_NAME_LENGTH + 1] = { '\0' };

  if (Opcode == PTC_INSTRUCTION_op_debug_insn_start) {
    // TODO: create accessors for PTC_INSTRUCTION_op_debug_insn_start
    uint64_t PC = Instruction.args[0];

    if (is64)
      PC |= Instruction.args[1] << 32;

    Result << " ---- 0x" << std::hex << PC << std::endl;
  } else if (Opcode == PTC_INSTRUCTION_op_call) {
    // TODO: replace PRIx64 with PTC_PRIxARG
    PTCInstructionArg FunctionPointer = 0;
    FunctionPointer = ptc_call_instruction_const_arg(&ptc, &Instruction, 0);
    PTCInstructionArg Flags = ptc_call_instruction_const_arg(&ptc,
                                                             &Instruction,
                                                             1);
    size_t OutArgsCount = ptc_call_instruction_out_arg_count(&ptc,
                                                             &Instruction);
    PTCHelperDef *Helper = ptc_find_helper(&ptc, FunctionPointer);
    const char *HelperName = "unknown_helper";

    if (Helper != nullptr && Helper->name != nullptr)
      HelperName = Helper->name;

    // The output format is:
    // call name, flags, out_args_count, out_args [...], in_args [...]
    Result << Definition->name << " " << HelperName << ","
           << "$0x" << std::hex << Flags << "," << std::dec << OutArgsCount;

    // Print out arguments
    for (i = 0; i < OutArgsCount; i++) {
      getTemporaryName(TemporaryName,
                       MAX_TEMP_NAME_LENGTH,
                       Instructions,
                       ptc_call_instruction_out_arg(&ptc, &Instruction, i));
      Result << "," << TemporaryName;
    }

    // Print in arguments
    size_t InArgsCount = ptc_call_instruction_in_arg_count(&ptc, &Instruction);
    for (i = 0; i < InArgsCount; i++) {
      PTCInstructionArg InArg = ptc_call_instruction_in_arg(&ptc,
                                                            &Instruction,
                                                            i);

      if (InArg != PTC_CALL_DUMMY_ARG) {
        getTemporaryName(TemporaryName,
                         MAX_TEMP_NAME_LENGTH,
                         Instructions,
                         InArg);
        Result << "," << TemporaryName;
      } else {
        Result << ",<dummy>";
      }
    }

  } else {
    // TODO: fix commas
    Result << Definition->name << " ";

    // Print out arguments
    for (i = 0; i < ptc_instruction_out_arg_count(&ptc, &Instruction); i++) {
      if (i != 0)
        Result << ",";

      getTemporaryName(TemporaryName,
                       MAX_TEMP_NAME_LENGTH,
                       Instructions,
                       ptc_instruction_out_arg(&ptc, &Instruction, i));
      Result << TemporaryName;
    }

    if (i != 0)
      Result << ",";

    // Print in arguments
    for (i = 0; i < ptc_instruction_in_arg_count(&ptc, &Instruction); i++) {
      if (i != 0)
        Result << ",";

      getTemporaryName(TemporaryName,
                       MAX_TEMP_NAME_LENGTH,
                       Instructions,
                       ptc_instruction_in_arg(&ptc, &Instruction, i));
      Result << TemporaryName;
    }

    if (i != 0)
      Result << ",";

    /* Parse some special const arguments */
    i = 0;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_brcond_i32:
    case PTC_INSTRUCTION_op_setcond_i32:
    case PTC_INSTRUCTION_op_movcond_i32:
    case PTC_INSTRUCTION_op_brcond2_i32:
    case PTC_INSTRUCTION_op_setcond2_i32:
    case PTC_INSTRUCTION_op_brcond_i64:
    case PTC_INSTRUCTION_op_setcond_i64:
    case PTC_INSTRUCTION_op_movcond_i64: {
      PTCInstructionArg Arg = ptc_instruction_const_arg(&ptc, &Instruction, 0);
      PTCCondition ConditionId = static_cast<PTCCondition>(Arg);
      const char *ConditionName = ptc.get_condition_name(ConditionId);

      if (ConditionName != nullptr)
        Result << "," << ConditionName;
      else
        Result << ","
               << "$0x" << std::hex << Arg;

      /* Consume one argument */
      i++;

    } break;
    case PTC_INSTRUCTION_op_qemu_ld_i32:
    case PTC_INSTRUCTION_op_qemu_st_i32:
    case PTC_INSTRUCTION_op_qemu_ld_i64:
    case PTC_INSTRUCTION_op_qemu_st_i64: {
      PTCInstructionArg Arg = ptc_instruction_const_arg(&ptc, &Instruction, 0);
      PTCLoadStoreArg LoadStoreArg = {};
      LoadStoreArg = ptc.parse_load_store_arg(Arg);

      if (LoadStoreArg.access_type == PTC_MEMORY_ACCESS_UNKNOWN)
        Result << ","
               << "$0x" << std::hex << LoadStoreArg.raw_op;
      else {
        const char *Alignment = nullptr;
        const char *LoadStoreName = nullptr;
        LoadStoreName = ptc.get_load_store_name(LoadStoreArg.type);

        switch (LoadStoreArg.access_type) {
        case PTC_MEMORY_ACCESS_NORMAL:
          Alignment = "";
          break;
        case PTC_MEMORY_ACCESS_UNALIGNED:
          Alignment = "un+";
          break;
        case PTC_MEMORY_ACCESS_ALIGNED:
          Alignment = "al+";
          break;
        default:
          return EXIT_FAILURE;
        }

        if (LoadStoreName == nullptr)
          return EXIT_FAILURE;

        Result << "," << Alignment << LoadStoreName;
      }

      Result << "," << LoadStoreArg.mmu_index;

      /* Consume one argument */
      i++;

    } break;
    default:
      break;
    }

    switch (Opcode) {
    case PTC_INSTRUCTION_op_set_label:
    case PTC_INSTRUCTION_op_br:
    case PTC_INSTRUCTION_op_brcond_i32:
    case PTC_INSTRUCTION_op_brcond_i64:
    case PTC_INSTRUCTION_op_brcond2_i32: {
      PTCInstructionArg Arg = ptc_instruction_const_arg(&ptc, &Instruction, i);
      Result << ","
             << "$L" << ptc.get_arg_label_id(Arg);

      /* Consume one more argument */
      i++;
      break;
    }
    default:
      break;
    }

    /* Print remaining const arguments */
    for (; i < ptc_instruction_const_arg_count(&ptc, &Instruction); i++) {
      if (i != 0) {
        Result << ",";
      }

      Result << "$0x" << std::hex
             << ptc_instruction_const_arg(&ptc, &Instruction, i);
    }
  }

  return EXIT_SUCCESS;
}

void disassemble(std::ostream &Result,
                 MetaAddress PC,
                 uint32_t MaxBytes,
                 uint32_t InstructionCount) {
  char *BufferPtr = nullptr;
  size_t BufferLenPtr = 0;
  FILE *MemoryStream = open_memstream(&BufferPtr, &BufferLenPtr);

  revng_assert(MemoryStream != nullptr);

  // Using SIZE_MAX is not very nice but the code should disassemble only a
  // single instruction nonetheless.
  ptc.disassemble(MemoryStream, PC.asPC(), MaxBytes, InstructionCount);
  fflush(MemoryStream);

  revng_assert(BufferPtr != nullptr);

  Result << BufferPtr;

  fclose(MemoryStream);
  free(BufferPtr);
}

int dumpTranslation(MetaAddress VirtualAddress,
                    std::ostream &Result,
                    PTCInstructionList *Instructions) {
  // TODO: this should stay in Architecture
  int is64 = 0;

  for (unsigned Index = 0; Index < Instructions->instruction_count; Index++) {
    PTCInstruction &Instruction = Instructions->instructions[Index];
    PTCOpcode Opcode = Instruction.opc;

    if (Opcode == PTC_INSTRUCTION_op_debug_insn_start) {
      uint64_t PC = Instruction.args[0];

      if (is64)
        PC |= Instruction.args[1] << 32;

      disassemble(Result, VirtualAddress.replaceAddress(PC), 4096, 1);
    }

    Result << std::dec << Index << ": ";

    if (dumpInstruction(Result, Instructions, Index) == EXIT_FAILURE)
      return EXIT_FAILURE;

    Result << std::endl;
  }

  return EXIT_SUCCESS;
}
