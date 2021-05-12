/// \file InstructionTranslator.cpp
/// \brief This file implements the logic to translate a PTC instruction in to
///        LLVM IR.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <cstdint>
#include <fstream>
#include <queue>
#include <set>
#include <sstream>

#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/BasicBlock.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Intrinsics.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Casting.h"

#include "revng/Lift/InstructionTranslator.h"
#include "revng/Lift/PTCInterface.h"
#include "revng/Lift/VariableManager.h"
#include "revng/Support/Assert.h"
#include "revng/Support/FunctionTags.h"
#include "revng/Support/IRHelpers.h"
#include "revng/Support/RandomAccessIterator.h"
#include "revng/Support/Range.h"

#include "InstructionTranslator.h"
#include "PTCInterface.h"
#include "VariableManager.h"

using namespace llvm;

static cl::opt<bool> RecordASM("record-asm",
                               cl::desc("create metadata for assembly"),
                               cl::cat(MainCategory));

using IT = InstructionTranslator;

namespace PTC {

template<bool C>
class InstructionImpl;

enum ArgumentType { In, Out, Const };

template<typename T, typename Q, bool B>
using RAI = RandomAccessIterator<T, Q, B>;

template<ArgumentType Type, bool IsCall>
class InstructionArgumentsIterator
  : public RAI<uint64_t, InstructionArgumentsIterator<Type, IsCall>, false> {

public:
  using base = RandomAccessIterator<uint64_t,
                                    InstructionArgumentsIterator,
                                    false>;

  InstructionArgumentsIterator &
  operator=(const InstructionArgumentsIterator &r) {
    base::operator=(r);
    TheInstruction = r.TheInstruction;
    return *this;
  }

  InstructionArgumentsIterator(const InstructionArgumentsIterator &r) :
    base(r), TheInstruction(r.TheInstruction) {}

  InstructionArgumentsIterator(const InstructionArgumentsIterator &r,
                               unsigned Index) :
    base(Index), TheInstruction(r.TheInstruction) {}

  InstructionArgumentsIterator(PTCInstruction *TheInstruction, unsigned Index) :
    base(Index), TheInstruction(TheInstruction) {}

  bool isCompatible(const InstructionArgumentsIterator &r) const {
    return TheInstruction == r.TheInstruction;
  }

public:
  uint64_t get(unsigned Index) const;

private:
  PTCInstruction *TheInstruction;
};

template<>
inline uint64_t
InstructionArgumentsIterator<In, true>::get(unsigned Index) const {
  return ptc_call_instruction_in_arg(&ptc, TheInstruction, Index);
}

template<>
inline uint64_t
InstructionArgumentsIterator<Const, true>::get(unsigned Index) const {
  return ptc_call_instruction_const_arg(&ptc, TheInstruction, Index);
}

template<>
inline uint64_t
InstructionArgumentsIterator<Out, true>::get(unsigned Index) const {
  return ptc_call_instruction_out_arg(&ptc, TheInstruction, Index);
}

template<>
inline uint64_t
InstructionArgumentsIterator<In, false>::get(unsigned Index) const {
  return ptc_instruction_in_arg(&ptc, TheInstruction, Index);
}

template<>
inline uint64_t
InstructionArgumentsIterator<Const, false>::get(unsigned Index) const {
  return ptc_instruction_const_arg(&ptc, TheInstruction, Index);
}

template<>
inline uint64_t
InstructionArgumentsIterator<Out, false>::get(unsigned Index) const {
  return ptc_instruction_out_arg(&ptc, TheInstruction, Index);
}

template<bool IsCall>
class InstructionImpl {
private:
  template<ArgumentType Type>
  using arguments = InstructionArgumentsIterator<Type, IsCall>;

public:
  InstructionImpl(PTCInstruction *TheInstruction) :
    TheInstruction(TheInstruction),
    InArguments(arguments<In>(TheInstruction, 0),
                arguments<In>(TheInstruction, inArgCount())),
    ConstArguments(arguments<Const>(TheInstruction, 0),
                   arguments<Const>(TheInstruction, constArgCount())),
    OutArguments(arguments<Out>(TheInstruction, 0),
                 arguments<Out>(TheInstruction, outArgCount())) {}

  PTCOpcode opcode() const { return TheInstruction->opc; }

  std::string helperName() const {
    revng_assert(IsCall);
    PTCHelperDef *Helper = ptc_find_helper(&ptc, ConstArguments[0]);
    revng_assert(Helper != nullptr && Helper->name != nullptr);
    return std::string(Helper->name);
  }

  uint64_t pc() const {
    revng_assert(opcode() == PTC_INSTRUCTION_op_debug_insn_start);
    uint64_t PC = ConstArguments[0];
    if (ConstArguments.size() > 1)
      PC |= ConstArguments[1] << 32;
    return PC;
  }

private:
  PTCInstruction *TheInstruction;

public:
  const Range<InstructionArgumentsIterator<In, IsCall>> InArguments;
  const Range<InstructionArgumentsIterator<Const, IsCall>> ConstArguments;
  const Range<InstructionArgumentsIterator<Out, IsCall>> OutArguments;

private:
  unsigned inArgCount() const;
  unsigned constArgCount() const;
  unsigned outArgCount() const;
};

using Instruction = InstructionImpl<false>;
using CallInstruction = InstructionImpl<true>;

template<>
inline unsigned CallInstruction::inArgCount() const {
  return ptc_call_instruction_in_arg_count(&ptc, TheInstruction);
}

template<>
inline unsigned Instruction::inArgCount() const {
  return ptc_instruction_in_arg_count(&ptc, TheInstruction);
}

template<>
inline unsigned CallInstruction::constArgCount() const {
  return ptc_call_instruction_const_arg_count(&ptc, TheInstruction);
}

template<>
inline unsigned Instruction::constArgCount() const {
  return ptc_instruction_const_arg_count(&ptc, TheInstruction);
}

template<>
inline unsigned CallInstruction::outArgCount() const {
  return ptc_call_instruction_out_arg_count(&ptc, TheInstruction);
}

template<>
inline unsigned Instruction::outArgCount() const {
  return ptc_instruction_out_arg_count(&ptc, TheInstruction);
}

} // namespace PTC

/// Converts a PTC condition into an LLVM predicate
///
/// \param Condition the input PTC condition.
///
/// \return the corresponding LLVM predicate.
static CmpInst::Predicate conditionToPredicate(PTCCondition Condition) {
  switch (Condition) {
  case PTC_COND_NEVER:
    // TODO: this is probably wrong
    return CmpInst::FCMP_FALSE;
  case PTC_COND_ALWAYS:
    // TODO: this is probably wrong
    return CmpInst::FCMP_TRUE;
  case PTC_COND_EQ:
    return CmpInst::ICMP_EQ;
  case PTC_COND_NE:
    return CmpInst::ICMP_NE;
  case PTC_COND_LT:
    return CmpInst::ICMP_SLT;
  case PTC_COND_GE:
    return CmpInst::ICMP_SGE;
  case PTC_COND_LE:
    return CmpInst::ICMP_SLE;
  case PTC_COND_GT:
    return CmpInst::ICMP_SGT;
  case PTC_COND_LTU:
    return CmpInst::ICMP_ULT;
  case PTC_COND_GEU:
    return CmpInst::ICMP_UGE;
  case PTC_COND_LEU:
    return CmpInst::ICMP_ULE;
  case PTC_COND_GTU:
    return CmpInst::ICMP_UGT;
  default:
    revng_unreachable("Unknown comparison operator");
  }
}

/// Obtains the LLVM binary operation corresponding to the specified PTC opcode.
///
/// \param Opcode the PTC opcode.
///
/// \return the LLVM binary operation matching opcode.
static Instruction::BinaryOps opcodeToBinaryOp(PTCOpcode Opcode) {
  switch (Opcode) {
  case PTC_INSTRUCTION_op_add_i32:
  case PTC_INSTRUCTION_op_add_i64:
  case PTC_INSTRUCTION_op_add2_i32:
  case PTC_INSTRUCTION_op_add2_i64:
    return Instruction::Add;
  case PTC_INSTRUCTION_op_sub_i32:
  case PTC_INSTRUCTION_op_sub_i64:
  case PTC_INSTRUCTION_op_sub2_i32:
  case PTC_INSTRUCTION_op_sub2_i64:
    return Instruction::Sub;
  case PTC_INSTRUCTION_op_mul_i32:
  case PTC_INSTRUCTION_op_mul_i64:
    return Instruction::Mul;
  case PTC_INSTRUCTION_op_div_i32:
  case PTC_INSTRUCTION_op_div_i64:
    return Instruction::SDiv;
  case PTC_INSTRUCTION_op_divu_i32:
  case PTC_INSTRUCTION_op_divu_i64:
    return Instruction::UDiv;
  case PTC_INSTRUCTION_op_rem_i32:
  case PTC_INSTRUCTION_op_rem_i64:
    return Instruction::SRem;
  case PTC_INSTRUCTION_op_remu_i32:
  case PTC_INSTRUCTION_op_remu_i64:
    return Instruction::URem;
  case PTC_INSTRUCTION_op_and_i32:
  case PTC_INSTRUCTION_op_and_i64:
    return Instruction::And;
  case PTC_INSTRUCTION_op_or_i32:
  case PTC_INSTRUCTION_op_or_i64:
    return Instruction::Or;
  case PTC_INSTRUCTION_op_xor_i32:
  case PTC_INSTRUCTION_op_xor_i64:
    return Instruction::Xor;
  case PTC_INSTRUCTION_op_shl_i32:
  case PTC_INSTRUCTION_op_shl_i64:
    return Instruction::Shl;
  case PTC_INSTRUCTION_op_shr_i32:
  case PTC_INSTRUCTION_op_shr_i64:
    return Instruction::LShr;
  case PTC_INSTRUCTION_op_sar_i32:
  case PTC_INSTRUCTION_op_sar_i64:
    return Instruction::AShr;
  default:
    revng_unreachable("PTC opcode is not a binary operator");
  }
}

/// Returns the maximum value which can be represented with the specified number
/// of bits.
static uint64_t getMaxValue(unsigned Bits) {
  if (Bits == 32)
    return 0xffffffff;
  else if (Bits == 64)
    return 0xffffffffffffffff;
  else
    revng_unreachable("Not the number of bits in a integer type");
}

/// Maps an opcode the corresponding input and output register size.
///
/// \return the size, in bits, of the registers used by the opcode.
static unsigned getRegisterSize(unsigned Opcode) {
  switch (Opcode) {
  case PTC_INSTRUCTION_op_add2_i32:
  case PTC_INSTRUCTION_op_add_i32:
  case PTC_INSTRUCTION_op_andc_i32:
  case PTC_INSTRUCTION_op_and_i32:
  case PTC_INSTRUCTION_op_brcond2_i32:
  case PTC_INSTRUCTION_op_brcond_i32:
  case PTC_INSTRUCTION_op_bswap16_i32:
  case PTC_INSTRUCTION_op_bswap32_i32:
  case PTC_INSTRUCTION_op_deposit_i32:
  case PTC_INSTRUCTION_op_div2_i32:
  case PTC_INSTRUCTION_op_div_i32:
  case PTC_INSTRUCTION_op_divu2_i32:
  case PTC_INSTRUCTION_op_divu_i32:
  case PTC_INSTRUCTION_op_eqv_i32:
  case PTC_INSTRUCTION_op_ext16s_i32:
  case PTC_INSTRUCTION_op_ext16u_i32:
  case PTC_INSTRUCTION_op_ext8s_i32:
  case PTC_INSTRUCTION_op_ext8u_i32:
  case PTC_INSTRUCTION_op_ld16s_i32:
  case PTC_INSTRUCTION_op_ld16u_i32:
  case PTC_INSTRUCTION_op_ld8s_i32:
  case PTC_INSTRUCTION_op_ld8u_i32:
  case PTC_INSTRUCTION_op_ld_i32:
  case PTC_INSTRUCTION_op_movcond_i32:
  case PTC_INSTRUCTION_op_mov_i32:
  case PTC_INSTRUCTION_op_movi_i32:
  case PTC_INSTRUCTION_op_mul_i32:
  case PTC_INSTRUCTION_op_muls2_i32:
  case PTC_INSTRUCTION_op_mulsh_i32:
  case PTC_INSTRUCTION_op_mulu2_i32:
  case PTC_INSTRUCTION_op_muluh_i32:
  case PTC_INSTRUCTION_op_nand_i32:
  case PTC_INSTRUCTION_op_neg_i32:
  case PTC_INSTRUCTION_op_nor_i32:
  case PTC_INSTRUCTION_op_not_i32:
  case PTC_INSTRUCTION_op_orc_i32:
  case PTC_INSTRUCTION_op_or_i32:
  case PTC_INSTRUCTION_op_qemu_ld_i32:
  case PTC_INSTRUCTION_op_qemu_st_i32:
  case PTC_INSTRUCTION_op_rem_i32:
  case PTC_INSTRUCTION_op_remu_i32:
  case PTC_INSTRUCTION_op_rotl_i32:
  case PTC_INSTRUCTION_op_rotr_i32:
  case PTC_INSTRUCTION_op_sar_i32:
  case PTC_INSTRUCTION_op_setcond2_i32:
  case PTC_INSTRUCTION_op_setcond_i32:
  case PTC_INSTRUCTION_op_shl_i32:
  case PTC_INSTRUCTION_op_shr_i32:
  case PTC_INSTRUCTION_op_st16_i32:
  case PTC_INSTRUCTION_op_st8_i32:
  case PTC_INSTRUCTION_op_st_i32:
  case PTC_INSTRUCTION_op_sub2_i32:
  case PTC_INSTRUCTION_op_sub_i32:
  case PTC_INSTRUCTION_op_trunc_shr_i32:
  case PTC_INSTRUCTION_op_xor_i32:
    return 32;
  case PTC_INSTRUCTION_op_add2_i64:
  case PTC_INSTRUCTION_op_add_i64:
  case PTC_INSTRUCTION_op_andc_i64:
  case PTC_INSTRUCTION_op_and_i64:
  case PTC_INSTRUCTION_op_brcond_i64:
  case PTC_INSTRUCTION_op_bswap16_i64:
  case PTC_INSTRUCTION_op_bswap32_i64:
  case PTC_INSTRUCTION_op_bswap64_i64:
  case PTC_INSTRUCTION_op_deposit_i64:
  case PTC_INSTRUCTION_op_div2_i64:
  case PTC_INSTRUCTION_op_div_i64:
  case PTC_INSTRUCTION_op_divu2_i64:
  case PTC_INSTRUCTION_op_divu_i64:
  case PTC_INSTRUCTION_op_eqv_i64:
  case PTC_INSTRUCTION_op_ext16s_i64:
  case PTC_INSTRUCTION_op_ext16u_i64:
  case PTC_INSTRUCTION_op_ext32s_i64:
  case PTC_INSTRUCTION_op_ext32u_i64:
  case PTC_INSTRUCTION_op_ext8s_i64:
  case PTC_INSTRUCTION_op_ext8u_i64:
  case PTC_INSTRUCTION_op_ld16s_i64:
  case PTC_INSTRUCTION_op_ld16u_i64:
  case PTC_INSTRUCTION_op_ld32s_i64:
  case PTC_INSTRUCTION_op_ld32u_i64:
  case PTC_INSTRUCTION_op_ld8s_i64:
  case PTC_INSTRUCTION_op_ld8u_i64:
  case PTC_INSTRUCTION_op_ld_i64:
  case PTC_INSTRUCTION_op_movcond_i64:
  case PTC_INSTRUCTION_op_mov_i64:
  case PTC_INSTRUCTION_op_movi_i64:
  case PTC_INSTRUCTION_op_mul_i64:
  case PTC_INSTRUCTION_op_muls2_i64:
  case PTC_INSTRUCTION_op_mulsh_i64:
  case PTC_INSTRUCTION_op_mulu2_i64:
  case PTC_INSTRUCTION_op_muluh_i64:
  case PTC_INSTRUCTION_op_nand_i64:
  case PTC_INSTRUCTION_op_neg_i64:
  case PTC_INSTRUCTION_op_nor_i64:
  case PTC_INSTRUCTION_op_not_i64:
  case PTC_INSTRUCTION_op_orc_i64:
  case PTC_INSTRUCTION_op_or_i64:
  case PTC_INSTRUCTION_op_qemu_ld_i64:
  case PTC_INSTRUCTION_op_qemu_st_i64:
  case PTC_INSTRUCTION_op_rem_i64:
  case PTC_INSTRUCTION_op_remu_i64:
  case PTC_INSTRUCTION_op_rotl_i64:
  case PTC_INSTRUCTION_op_rotr_i64:
  case PTC_INSTRUCTION_op_sar_i64:
  case PTC_INSTRUCTION_op_setcond_i64:
  case PTC_INSTRUCTION_op_shl_i64:
  case PTC_INSTRUCTION_op_shr_i64:
  case PTC_INSTRUCTION_op_st16_i64:
  case PTC_INSTRUCTION_op_st32_i64:
  case PTC_INSTRUCTION_op_st8_i64:
  case PTC_INSTRUCTION_op_st_i64:
  case PTC_INSTRUCTION_op_sub2_i64:
  case PTC_INSTRUCTION_op_sub_i64:
  case PTC_INSTRUCTION_op_xor_i64:
    return 64;
  case PTC_INSTRUCTION_op_br:
  case PTC_INSTRUCTION_op_call:
  case PTC_INSTRUCTION_op_debug_insn_start:
  case PTC_INSTRUCTION_op_discard:
  case PTC_INSTRUCTION_op_exit_tb:
  case PTC_INSTRUCTION_op_goto_tb:
  case PTC_INSTRUCTION_op_set_label:
    return 0;
  default:
    revng_unreachable("Unexpected opcode");
  }
}

/// Create a compare instruction given a comparison operator and the operands
///
/// \param Builder the builder to use to create the instruction.
/// \param RawCondition the PTC condition.
/// \param FirstOperand the first operand of the comparison.
/// \param SecondOperand the second operand of the comparison.
///
/// \return a compare instruction.
template<typename T>
static Value *CreateICmp(T &Builder,
                         uint64_t RawCondition,
                         Value *FirstOperand,
                         Value *SecondOperand) {
  PTCCondition Condition = static_cast<PTCCondition>(RawCondition);
  return Builder.CreateICmp(conditionToPredicate(Condition),
                            FirstOperand,
                            SecondOperand);
}

using LBM = IT::LabeledBlocksMap;
IT::InstructionTranslator(IRBuilder<> &Builder,
                          VariableManager &Variables,
                          JumpTargetManager &JumpTargets,
                          std::vector<BasicBlock *> Blocks,
                          const Architecture &SourceArchitecture,
                          const Architecture &TargetArchitecture,
                          ProgramCounterHandler *PCH) :
  Builder(Builder),
  Variables(Variables),
  JumpTargets(JumpTargets),
  Blocks(Blocks),
  TheModule(*Builder.GetInsertBlock()->getParent()->getParent()),
  TheFunction(Builder.GetInsertBlock()->getParent()),
  SourceArchitecture(SourceArchitecture),
  TargetArchitecture(TargetArchitecture),
  NewPCMarker(nullptr),
  LastPC(MetaAddress::invalid()),
  MetaAddressStruct(MetaAddress::getStruct(&TheModule)),
  PCH(PCH) {

  auto &Context = TheModule.getContext();
  using FT = FunctionType;
  // The newpc function call takes the following parameters:
  //
  // * address of the instruction
  // * instruction size
  // * isJT (-1: unknown, 0: no, 1: yes)
  // * pointer to the disassembled instruction
  // * all the local variables used by this instruction
  auto *NewPCMarkerTy = FT::get(Type::getVoidTy(Context),
                                { MetaAddressStruct,
                                  Type::getInt64Ty(Context),
                                  Type::getInt32Ty(Context),
                                  Type::getInt8PtrTy(Context) },
                                true);
  NewPCMarker = Function::Create(NewPCMarkerTy,
                                 GlobalValue::ExternalLinkage,
                                 "newpc",
                                 &TheModule);
  FunctionTags::Marker.addTo(NewPCMarker);
}

void IT::finalizeNewPCMarkers(std::string &CoveragePath) {
  std::ofstream Output(CoveragePath);

  Output << std::hex;
  size_t FixedArgCount = NewPCMarker->arg_size();

  llvm::SmallVector<CallInst *, 4> CallsToRemove;

  for (User *U : NewPCMarker->users()) {
    auto *Call = cast<CallInst>(U);

    // Report the instruction on the coverage CSV
    auto PC = MetaAddress::fromConstant(Call->getArgOperand(0));
    uint64_t Size = getLimitedValue(Call->getArgOperand(1));
    bool IsJT = JumpTargets.isJumpTarget(PC);
    PC.dump(Output);
    Output << ",0x" << Size << "," << (IsJT ? "1" : "0") << "\n";

    // We already finished discovering new code to translate, so we can remove
    // the references to local variables as argument of the calls to newpc and
    // create room for more optimizations.
    if (Call->arg_size() != FixedArgCount) {
      SmallVector<Value *, 8> Args;
      auto AI = Call->arg_begin();
      for (size_t Idx = 0; Idx < FixedArgCount; ++Idx, ++AI)
        Args.emplace_back(*AI);

      auto *NewCall = CallInst::Create(NewPCMarker, Args, "", Call);
      NewCall->setCallingConv(Call->getCallingConv());
      NewCall->setAttributes(Call->getAttributes());
      NewCall->setDebugLoc(Call->getDebugLoc());
      NewCall->copyMetadata(*Call);

      revng_assert(Call->use_empty());
      CallsToRemove.push_back(Call);
    }
  }

  for (auto *Call : CallsToRemove)
    Call->eraseFromParent();

  Output << std::dec;
}

SmallSet<unsigned, 1> IT::preprocess(PTCInstructionList *InstructionList) {
  SmallSet<unsigned, 1> Result;

  for (unsigned I = 0; I < InstructionList->instruction_count; I++) {
    PTCInstruction &Instruction = InstructionList->instructions[I];
    switch (Instruction.opc) {
    case PTC_INSTRUCTION_op_movi_i32:
    case PTC_INSTRUCTION_op_movi_i64:
    case PTC_INSTRUCTION_op_mov_i32:
    case PTC_INSTRUCTION_op_mov_i64:
      break;
    default:
      continue;
    }

    const PTC::Instruction TheInstruction(&Instruction);
    unsigned OutArg = TheInstruction.OutArguments[0];
    PTCTemp *Temporary = ptc_temp_get(InstructionList, OutArg);

    if (!ptc_temp_is_global(InstructionList, OutArg))
      continue;

    if (0 != strcmp("btarget", Temporary->name))
      continue;

    for (unsigned J = I + 1; J < InstructionList->instruction_count; J++) {
      unsigned Opcode = InstructionList->instructions[J].opc;
      if (Opcode == PTC_INSTRUCTION_op_debug_insn_start)
        Result.insert(J);
    }

    break;
  }

  return Result;
}

std::tuple<IT::TranslationResult, MDNode *, MetaAddress, MetaAddress>
IT::newInstruction(PTCInstruction *Instr,
                   PTCInstruction *Next,
                   MetaAddress StartPC,
                   MetaAddress EndPC,
                   bool IsFirst,
                   MetaAddress AbortAt) {
  using R = std::tuple<TranslationResult, MDNode *, MetaAddress, MetaAddress>;
  revng_assert(Instr != nullptr);

  LLVMContext &Context = TheModule.getContext();

  const PTC::Instruction TheInstruction(Instr);
  // A new original instruction, let's create a new metadata node
  // referencing it for all the next instructions to come
  MetaAddress PC = StartPC.replaceAddress(TheInstruction.pc());
  MetaAddress NextPC = MetaAddress::invalid();

  if (Next != nullptr)
    NextPC = StartPC.replaceAddress(PTC::Instruction(Next).pc());
  else
    NextPC = EndPC;

  if (AbortAt.isValid() and NextPC.addressGreaterThan(AbortAt))
    return R{ Abort, nullptr, MetaAddress::invalid(), MetaAddress::invalid() };

  MDNode *MDOriginalInstr = nullptr;
  Constant *String = nullptr;
  if (RecordASM) {
    std::stringstream OriginalStringStream;
    disassemble(OriginalStringStream, PC, NextPC - PC);
    std::string OriginalString = OriginalStringStream.str();

    // We don't deduplicate this string since performing a lookup each time is
    // increasingly expensive and we should have relatively few collisions
    std::string AddressName = JumpTargets.nameForAddress(PC);
    String = buildStringPtr(&TheModule,
                            OriginalString,
                            Twine("disam_") + AddressName);

    auto *MDOriginalString = ConstantAsMetadata::get(String);
    auto *MDPC = ConstantAsMetadata::get(PC.toConstant(MetaAddressStruct));
    MDOriginalInstr = MDNode::get(Context, { MDOriginalString, MDPC });
  } else {
    String = ConstantPointerNull::get(getStringPtrType(Context));
  }

  if (!IsFirst) {
    // Check if this PC already has a block and use it
    bool ShouldContinue;
    BasicBlock *DivergeTo = JumpTargets.newPC(PC, ShouldContinue);
    if (DivergeTo != nullptr) {
      Builder.CreateBr(DivergeTo);

      if (ShouldContinue) {
        // The block is empty, let's fill it
        Blocks.push_back(DivergeTo);
        Builder.SetInsertPoint(DivergeTo);
      } else {
        // The block contains already translated code, early exit
        return R{ Stop, MDOriginalInstr, PC, NextPC };
      }
    }
  }

  Variables.newBasicBlock();

  // Insert a call to NewPCMarker capturing all the local temporaries
  // This prevents SROA from transforming them in SSA values, which is bad
  // in case we have to split a basic block
  revng_assert(MetaAddressStruct != nullptr);
  std::vector<Value *> Args = { PC.toConstant(MetaAddressStruct),
                                Builder.getInt64(NextPC - PC),
                                Builder.getInt32(-1),
                                String };
  for (AllocaInst *Local : Variables.locals())
    Args.push_back(Local);

  auto *Call = Builder.CreateCall(NewPCMarker, Args);

  if (!IsFirst) {
    // Inform the JumpTargetManager about the new PC we met
    BasicBlock::iterator CurrentIt = Builder.GetInsertPoint();
    if (CurrentIt == Builder.GetInsertBlock()->begin())
      revng_assert(JumpTargets.getBlockAt(PC) == Builder.GetInsertBlock());
    else
      JumpTargets.registerInstruction(PC, Call);
  }

  return R{ Success, MDOriginalInstr, PC, NextPC };
}

IT::TranslationResult IT::translateCall(PTCInstruction *Instr) {
  const PTC::CallInstruction TheCall(Instr);

  std::vector<Value *> InArgs;

  for (uint64_t TemporaryId : TheCall.InArguments) {
    auto *Load = Variables.load(Builder, TemporaryId);
    if (Load == nullptr)
      return Abort;
    InArgs.push_back(Load);
  }

  const auto GetValueType = [](Value *Argument) { return Argument->getType(); };
  auto ValueTypes = llvm::map_range(InArgs, GetValueType);
  std::vector<Type *> InArgsType(ValueTypes.begin(), ValueTypes.end());

  // TODO: handle multiple return arguments
  revng_assert(TheCall.OutArguments.size() <= 1);

  Value *ResultDestination = nullptr;
  Type *ResultType = nullptr;

  if (TheCall.OutArguments.size() != 0) {
    ResultDestination = Variables.getOrCreate(TheCall.OutArguments[0]);
    if (ResultDestination == nullptr)
      return Abort;
    ResultType = ResultDestination->getType()->getPointerElementType();
  } else {
    ResultType = Builder.getVoidTy();
  }

  auto *CalleeType = FunctionType::get(ResultType,
                                       ArrayRef<Type *>(InArgsType),
                                       false);

  std::string HelperName = "helper_" + TheCall.helperName();
  FunctionCallee FDecl = TheModule.getOrInsertFunction(HelperName, CalleeType);

  FunctionTags::Helper.addTo(cast<Function>(skipCasts(FDecl.getCallee())));

  CallInst *Result = Builder.CreateCall(FDecl, InArgs);

  if (TheCall.OutArguments.size() != 0)
    Builder.CreateStore(Result, ResultDestination);

  return Success;
}

IT::TranslationResult
IT::translate(PTCInstruction *Instr, MetaAddress PC, MetaAddress NextPC) {
  const PTC::Instruction TheInstruction(Instr);

  std::vector<Value *> InArgs;
  for (uint64_t TemporaryId : TheInstruction.InArguments) {
    auto *Load = Variables.load(Builder, TemporaryId);
    if (Load == nullptr)
      return Abort;
    InArgs.push_back(Load);
  }

  auto ConstArgs = TheInstruction.ConstArguments;
  LastPC = PC;
  auto Result = translateOpcode(TheInstruction.opcode(),
                                ConstArgs.toVector(),
                                InArgs);

  // Check if there was an error while translating the instruction
  if (!Result)
    return Abort;

  size_t OutSize = TheInstruction.OutArguments.size();
  revng_assert(Result->size() == OutSize);

  // TODO: use ZipIterator here
  for (unsigned I = 0; I < Result->size(); I++) {
    auto *Destination = Variables.getOrCreate(TheInstruction.OutArguments[I]);
    if (Destination == nullptr)
      return Abort;

    auto *Store = Builder.CreateStore(Result.get()[I], Destination);

    if (PCH->affectsPC(Store)) {
      // This is a PC-related store
      PCH->handleStore(Builder, Store);
    } else {
      // If we're writing somewhere an immediate, register it for exploration
      if (auto *Constant = dyn_cast<ConstantInt>(Store->getValueOperand())) {
        MetaAddress Address = JumpTargets.fromPC(Constant->getLimitedValue());
        if (Address.isValid() and PC != Address and JumpTargets.isPC(Address)
            and not JumpTargets.hasJT(Address)) {
          JumpTargets.registerSimpleLiteral(Address);
        }
      }
    }
  }

  return Success;
}

void IT::registerDirectJumps() {

  for (BasicBlock *ExitBB : ExitBlocks) {
    auto [Result, NextPC] = PCH->getUniqueJumpTarget(ExitBB);
    if (Result == NextJumpTarget::Unique and JumpTargets.isPC(NextPC)
        and not JumpTargets.hasJT(NextPC)) {
      JumpTargets.registerJT(NextPC, JTReason::DirectJump);
    }
  }

  ExitBlocks.clear();
}

ErrorOr<std::vector<Value *>>
IT::translateOpcode(PTCOpcode Opcode,
                    std::vector<uint64_t> ConstArguments,
                    std::vector<Value *> InArguments) {
  LLVMContext &Context = TheModule.getContext();
  unsigned RegisterSize = getRegisterSize(Opcode);
  Type *RegisterType = nullptr;
  if (RegisterSize == 32)
    RegisterType = Builder.getInt32Ty();
  else if (RegisterSize == 64)
    RegisterType = Builder.getInt64Ty();
  else if (RegisterSize != 0)
    revng_unreachable("Unexpected register size");

  using v = std::vector<Value *>;
  switch (Opcode) {
  case PTC_INSTRUCTION_op_movi_i32:
  case PTC_INSTRUCTION_op_movi_i64:
    return v{ ConstantInt::get(RegisterType, ConstArguments[0]) };
  case PTC_INSTRUCTION_op_discard:
    // Let's overwrite the discarded temporary with a 0
    return v{ ConstantInt::get(RegisterType, 0) };
  case PTC_INSTRUCTION_op_mov_i32:
  case PTC_INSTRUCTION_op_mov_i64:
    return v{ Builder.CreateTrunc(InArguments[0], RegisterType) };
  case PTC_INSTRUCTION_op_setcond_i32:
  case PTC_INSTRUCTION_op_setcond_i64: {
    Value *Compare = CreateICmp(Builder,
                                ConstArguments[0],
                                InArguments[0],
                                InArguments[1]);
    // TODO: convert single-bit registers to i1
    return v{ Builder.CreateZExt(Compare, RegisterType) };
  }
  case PTC_INSTRUCTION_op_movcond_i32: // Resist the fallthrough temptation
  case PTC_INSTRUCTION_op_movcond_i64: {
    Value *Compare = CreateICmp(Builder,
                                ConstArguments[0],
                                InArguments[0],
                                InArguments[1]);
    Value *Select = Builder.CreateSelect(Compare,
                                         InArguments[2],
                                         InArguments[3]);
    return v{ Select };
  }
  case PTC_INSTRUCTION_op_qemu_ld_i32:
  case PTC_INSTRUCTION_op_qemu_ld_i64:
  case PTC_INSTRUCTION_op_qemu_st_i32:
  case PTC_INSTRUCTION_op_qemu_st_i64: {
    PTCLoadStoreArg MemoryAccess;
    MemoryAccess = ptc.parse_load_store_arg(ConstArguments[0]);

    // What are we supposed to do in this case?
    revng_assert(MemoryAccess.access_type != PTC_MEMORY_ACCESS_UNKNOWN);

    unsigned Alignment = 0;
    if (MemoryAccess.access_type == PTC_MEMORY_ACCESS_UNALIGNED)
      Alignment = 1;
    else
      Alignment = SourceArchitecture.defaultAlignment();

    // Load size
    IntegerType *MemoryType = nullptr;
    switch (ptc_get_memory_access_size(MemoryAccess.type)) {
    case PTC_MO_8:
      MemoryType = Builder.getInt8Ty();
      break;
    case PTC_MO_16:
      MemoryType = Builder.getInt16Ty();
      break;
    case PTC_MO_32:
      MemoryType = Builder.getInt32Ty();
      break;
    case PTC_MO_64:
      MemoryType = Builder.getInt64Ty();
      break;
    default:
      revng_unreachable("Unexpected load size");
    }

    // If necessary, handle endianess mismatch
    // TODO: it might be a bit overkill, but it be nice to make this function
    //       template-parametric w.r.t. endianess mismatch
    Function *BSwapFunction = nullptr;
    if (MemoryType != Builder.getInt8Ty()
        && SourceArchitecture.endianess() != TargetArchitecture.endianess())
      BSwapFunction = Intrinsic::getDeclaration(&TheModule,
                                                Intrinsic::bswap,
                                                { MemoryType });

    bool SignExtend = ptc_is_sign_extended_load(MemoryAccess.type);

    Value *Pointer = nullptr;
    if (Opcode == PTC_INSTRUCTION_op_qemu_ld_i32
        || Opcode == PTC_INSTRUCTION_op_qemu_ld_i64) {

      Pointer = Builder.CreateIntToPtr(InArguments[0],
                                       MemoryType->getPointerTo());
      auto *Load = Builder.CreateAlignedLoad(Pointer, MaybeAlign(Alignment));
      Value *Loaded = Load;

      if (BSwapFunction != nullptr)
        Loaded = Builder.CreateCall(BSwapFunction, Load);

      if (SignExtend)
        return v{ Builder.CreateSExt(Loaded, RegisterType) };
      else
        return v{ Builder.CreateZExt(Loaded, RegisterType) };

    } else if (Opcode == PTC_INSTRUCTION_op_qemu_st_i32
               || Opcode == PTC_INSTRUCTION_op_qemu_st_i64) {

      Pointer = Builder.CreateIntToPtr(InArguments[1],
                                       MemoryType->getPointerTo());
      Value *Value = Builder.CreateTrunc(InArguments[0], MemoryType);

      if (BSwapFunction != nullptr)
        Value = Builder.CreateCall(BSwapFunction, Value);

      Builder.CreateAlignedStore(Value, Pointer, MaybeAlign(Alignment));

      return v{};
    } else {
      revng_unreachable("Unknown load type");
    }
  }
  case PTC_INSTRUCTION_op_ld8u_i32:
  case PTC_INSTRUCTION_op_ld8s_i32:
  case PTC_INSTRUCTION_op_ld16u_i32:
  case PTC_INSTRUCTION_op_ld16s_i32:
  case PTC_INSTRUCTION_op_ld_i32:
  case PTC_INSTRUCTION_op_ld8u_i64:
  case PTC_INSTRUCTION_op_ld8s_i64:
  case PTC_INSTRUCTION_op_ld16u_i64:
  case PTC_INSTRUCTION_op_ld16s_i64:
  case PTC_INSTRUCTION_op_ld32u_i64:
  case PTC_INSTRUCTION_op_ld32s_i64:
  case PTC_INSTRUCTION_op_ld_i64: {
    Value *Base = dyn_cast<LoadInst>(InArguments[0])->getPointerOperand();
    if (Base == nullptr || !Variables.isEnv(Base)) {
      // TODO: emit warning
      return std::errc::invalid_argument;
    }

    bool Signed;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_ld_i32:
    case PTC_INSTRUCTION_op_ld_i64:

    case PTC_INSTRUCTION_op_ld8u_i32:
    case PTC_INSTRUCTION_op_ld16u_i32:
    case PTC_INSTRUCTION_op_ld8u_i64:
    case PTC_INSTRUCTION_op_ld16u_i64:
    case PTC_INSTRUCTION_op_ld32u_i64:
      Signed = false;
      break;
    case PTC_INSTRUCTION_op_ld8s_i32:
    case PTC_INSTRUCTION_op_ld16s_i32:
    case PTC_INSTRUCTION_op_ld8s_i64:
    case PTC_INSTRUCTION_op_ld16s_i64:
    case PTC_INSTRUCTION_op_ld32s_i64:
      Signed = true;
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    unsigned LoadSize;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_ld8u_i32:
    case PTC_INSTRUCTION_op_ld8s_i32:
    case PTC_INSTRUCTION_op_ld8u_i64:
    case PTC_INSTRUCTION_op_ld8s_i64:
      LoadSize = 1;
      break;
    case PTC_INSTRUCTION_op_ld16u_i32:
    case PTC_INSTRUCTION_op_ld16s_i32:
    case PTC_INSTRUCTION_op_ld16u_i64:
    case PTC_INSTRUCTION_op_ld16s_i64:
      LoadSize = 2;
      break;
    case PTC_INSTRUCTION_op_ld_i32:
    case PTC_INSTRUCTION_op_ld32u_i64:
    case PTC_INSTRUCTION_op_ld32s_i64:
      LoadSize = 4;
      break;
    case PTC_INSTRUCTION_op_ld_i64:
      LoadSize = 8;
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    Value *Result = Variables.loadFromEnvOffset(Builder,
                                                LoadSize,
                                                ConstArguments[0]);
    revng_assert(Result != nullptr);

    // Zero/sign extend in the target dimension
    if (Signed)
      return v{ Builder.CreateSExt(Result, RegisterType) };
    else
      return v{ Builder.CreateZExt(Result, RegisterType) };
  }
  case PTC_INSTRUCTION_op_st8_i32:
  case PTC_INSTRUCTION_op_st16_i32:
  case PTC_INSTRUCTION_op_st_i32:
  case PTC_INSTRUCTION_op_st8_i64:
  case PTC_INSTRUCTION_op_st16_i64:
  case PTC_INSTRUCTION_op_st32_i64:
  case PTC_INSTRUCTION_op_st_i64: {
    unsigned StoreSize;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_st8_i32:
    case PTC_INSTRUCTION_op_st8_i64:
      StoreSize = 1;
      break;
    case PTC_INSTRUCTION_op_st16_i32:
    case PTC_INSTRUCTION_op_st16_i64:
      StoreSize = 2;
      break;
    case PTC_INSTRUCTION_op_st_i32:
    case PTC_INSTRUCTION_op_st32_i64:
      StoreSize = 4;
      break;
    case PTC_INSTRUCTION_op_st_i64:
      StoreSize = 8;
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    Value *Base = dyn_cast<LoadInst>(InArguments[1])->getPointerOperand();
    if (Base == nullptr || !Variables.isEnv(Base)) {
      // TODO: emit warning
      return std::errc::invalid_argument;
    }

    auto Result = Variables.storeToEnvOffset(Builder,
                                             StoreSize,
                                             ConstArguments[0],
                                             InArguments[0]);
    PCH->handleStore(Builder, *Result);

    return v{};
  }
  case PTC_INSTRUCTION_op_add_i32:
  case PTC_INSTRUCTION_op_sub_i32:
  case PTC_INSTRUCTION_op_mul_i32:
  case PTC_INSTRUCTION_op_div_i32:
  case PTC_INSTRUCTION_op_divu_i32:
  case PTC_INSTRUCTION_op_rem_i32:
  case PTC_INSTRUCTION_op_remu_i32:
  case PTC_INSTRUCTION_op_and_i32:
  case PTC_INSTRUCTION_op_or_i32:
  case PTC_INSTRUCTION_op_xor_i32:
  case PTC_INSTRUCTION_op_shl_i32:
  case PTC_INSTRUCTION_op_shr_i32:
  case PTC_INSTRUCTION_op_sar_i32:
  case PTC_INSTRUCTION_op_add_i64:
  case PTC_INSTRUCTION_op_sub_i64:
  case PTC_INSTRUCTION_op_mul_i64:
  case PTC_INSTRUCTION_op_div_i64:
  case PTC_INSTRUCTION_op_divu_i64:
  case PTC_INSTRUCTION_op_rem_i64:
  case PTC_INSTRUCTION_op_remu_i64:
  case PTC_INSTRUCTION_op_and_i64:
  case PTC_INSTRUCTION_op_or_i64:
  case PTC_INSTRUCTION_op_xor_i64:
  case PTC_INSTRUCTION_op_shl_i64:
  case PTC_INSTRUCTION_op_shr_i64:
  case PTC_INSTRUCTION_op_sar_i64: {
    // TODO: assert on sizes?
    Instruction::BinaryOps BinaryOp = opcodeToBinaryOp(Opcode);
    Value *Operation = Builder.CreateBinOp(BinaryOp,
                                           InArguments[0],
                                           InArguments[1]);
    return v{ Operation };
  }
  case PTC_INSTRUCTION_op_div2_i32:
  case PTC_INSTRUCTION_op_divu2_i32:
  case PTC_INSTRUCTION_op_div2_i64:
  case PTC_INSTRUCTION_op_divu2_i64: {
    Instruction::BinaryOps DivisionOp, RemainderOp;

    if (Opcode == PTC_INSTRUCTION_op_div2_i32
        || Opcode == PTC_INSTRUCTION_op_div2_i64) {
      DivisionOp = Instruction::SDiv;
      RemainderOp = Instruction::SRem;
    } else if (Opcode == PTC_INSTRUCTION_op_divu2_i32
               || Opcode == PTC_INSTRUCTION_op_divu2_i64) {
      DivisionOp = Instruction::UDiv;
      RemainderOp = Instruction::URem;
    } else {
      revng_unreachable("Unknown operation type");
    }

    // TODO: we're ignoring InArguments[1], which is the MSB
    // TODO: assert on sizes?
    Value *Division = Builder.CreateBinOp(DivisionOp,
                                          InArguments[0],
                                          InArguments[2]);
    Value *Remainder = Builder.CreateBinOp(RemainderOp,
                                           InArguments[0],
                                           InArguments[2]);
    return v{ Division, Remainder };
  }
  case PTC_INSTRUCTION_op_rotr_i32:
  case PTC_INSTRUCTION_op_rotr_i64:
  case PTC_INSTRUCTION_op_rotl_i32:
  case PTC_INSTRUCTION_op_rotl_i64: {
    Value *Bits = ConstantInt::get(RegisterType, RegisterSize);

    Instruction::BinaryOps FirstShiftOp, SecondShiftOp;
    if (Opcode == PTC_INSTRUCTION_op_rotl_i32
        || Opcode == PTC_INSTRUCTION_op_rotl_i64) {
      FirstShiftOp = Instruction::Shl;
      SecondShiftOp = Instruction::LShr;
    } else if (Opcode == PTC_INSTRUCTION_op_rotr_i32
               || Opcode == PTC_INSTRUCTION_op_rotr_i64) {
      FirstShiftOp = Instruction::LShr;
      SecondShiftOp = Instruction::Shl;
    } else {
      revng_unreachable("Unexpected opcode");
    }

    Value *FirstShift = Builder.CreateBinOp(FirstShiftOp,
                                            InArguments[0],
                                            InArguments[1]);
    Value *SecondShiftAmount = Builder.CreateSub(Bits, InArguments[1]);
    Value *SecondShift = Builder.CreateBinOp(SecondShiftOp,
                                             InArguments[0],
                                             SecondShiftAmount);

    return v{ Builder.CreateOr(FirstShift, SecondShift) };
  }
  case PTC_INSTRUCTION_op_deposit_i32:
  case PTC_INSTRUCTION_op_deposit_i64: {
    unsigned Position = ConstArguments[0];
    if (Position == RegisterSize)
      return v{ InArguments[0] };

    unsigned Length = ConstArguments[1];
    uint64_t Bits = 0;

    // Thou shall not << 32
    if (Length == RegisterSize)
      Bits = getMaxValue(RegisterSize);
    else
      Bits = (1 << Length) - 1;

    // result = (t1 & ~(bits << position)) | ((t2 & bits) << position)
    uint64_t BaseMask = ~(Bits << Position);
    Value *MaskedBase = Builder.CreateAnd(InArguments[0], BaseMask);
    Value *Deposit = Builder.CreateAnd(InArguments[1], Bits);
    Value *ShiftedDeposit = Builder.CreateShl(Deposit, Position);
    Value *Result = Builder.CreateOr(MaskedBase, ShiftedDeposit);

    return v{ Result };
  }
  case PTC_INSTRUCTION_op_ext8s_i32:
  case PTC_INSTRUCTION_op_ext16s_i32:
  case PTC_INSTRUCTION_op_ext8u_i32:
  case PTC_INSTRUCTION_op_ext16u_i32:
  case PTC_INSTRUCTION_op_ext8s_i64:
  case PTC_INSTRUCTION_op_ext16s_i64:
  case PTC_INSTRUCTION_op_ext32s_i64:
  case PTC_INSTRUCTION_op_ext8u_i64:
  case PTC_INSTRUCTION_op_ext16u_i64:
  case PTC_INSTRUCTION_op_ext32u_i64: {
    Type *SourceType = nullptr;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_ext8s_i32:
    case PTC_INSTRUCTION_op_ext8u_i32:
    case PTC_INSTRUCTION_op_ext8s_i64:
    case PTC_INSTRUCTION_op_ext8u_i64:
      SourceType = Builder.getInt8Ty();
      break;
    case PTC_INSTRUCTION_op_ext16s_i32:
    case PTC_INSTRUCTION_op_ext16u_i32:
    case PTC_INSTRUCTION_op_ext16s_i64:
    case PTC_INSTRUCTION_op_ext16u_i64:
      SourceType = Builder.getInt16Ty();
      break;
    case PTC_INSTRUCTION_op_ext32s_i64:
    case PTC_INSTRUCTION_op_ext32u_i64:
      SourceType = Builder.getInt32Ty();
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    Value *Truncated = Builder.CreateTrunc(InArguments[0], SourceType);

    switch (Opcode) {
    case PTC_INSTRUCTION_op_ext8s_i32:
    case PTC_INSTRUCTION_op_ext8s_i64:
    case PTC_INSTRUCTION_op_ext16s_i32:
    case PTC_INSTRUCTION_op_ext16s_i64:
    case PTC_INSTRUCTION_op_ext32s_i64:
      return v{ Builder.CreateSExt(Truncated, RegisterType) };
    case PTC_INSTRUCTION_op_ext8u_i32:
    case PTC_INSTRUCTION_op_ext8u_i64:
    case PTC_INSTRUCTION_op_ext16u_i32:
    case PTC_INSTRUCTION_op_ext16u_i64:
    case PTC_INSTRUCTION_op_ext32u_i64:
      return v{ Builder.CreateZExt(Truncated, RegisterType) };
    default:
      revng_unreachable("Unexpected opcode");
    }
  }
  case PTC_INSTRUCTION_op_not_i32:
  case PTC_INSTRUCTION_op_not_i64:
    return v{ Builder.CreateXor(InArguments[0], getMaxValue(RegisterSize)) };
  case PTC_INSTRUCTION_op_neg_i32:
  case PTC_INSTRUCTION_op_neg_i64: {
    auto *InitialValue = ConstantInt::get(RegisterType, 0);
    return v{ Builder.CreateSub(InitialValue, InArguments[0]) };
  }
  case PTC_INSTRUCTION_op_andc_i32:
  case PTC_INSTRUCTION_op_andc_i64:
  case PTC_INSTRUCTION_op_orc_i32:
  case PTC_INSTRUCTION_op_orc_i64:
  case PTC_INSTRUCTION_op_eqv_i32:
  case PTC_INSTRUCTION_op_eqv_i64: {
    Instruction::BinaryOps ExternalOp;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_andc_i32:
    case PTC_INSTRUCTION_op_andc_i64:
      ExternalOp = Instruction::And;
      break;
    case PTC_INSTRUCTION_op_orc_i32:
    case PTC_INSTRUCTION_op_orc_i64:
      ExternalOp = Instruction::Or;
      break;
    case PTC_INSTRUCTION_op_eqv_i32:
    case PTC_INSTRUCTION_op_eqv_i64:
      ExternalOp = Instruction::Xor;
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    Value *Negate = Builder.CreateXor(InArguments[1],
                                      getMaxValue(RegisterSize));
    Value *Result = Builder.CreateBinOp(ExternalOp, InArguments[0], Negate);
    return v{ Result };
  }
  case PTC_INSTRUCTION_op_nand_i32:
  case PTC_INSTRUCTION_op_nand_i64: {
    Value *AndValue = Builder.CreateAnd(InArguments[0], InArguments[1]);
    Value *Result = Builder.CreateXor(AndValue, getMaxValue(RegisterSize));
    return v{ Result };
  }
  case PTC_INSTRUCTION_op_nor_i32:
  case PTC_INSTRUCTION_op_nor_i64: {
    Value *OrValue = Builder.CreateOr(InArguments[0], InArguments[1]);
    Value *Result = Builder.CreateXor(OrValue, getMaxValue(RegisterSize));
    return v{ Result };
  }
  case PTC_INSTRUCTION_op_bswap16_i32:
  case PTC_INSTRUCTION_op_bswap32_i32:
  case PTC_INSTRUCTION_op_bswap16_i64:
  case PTC_INSTRUCTION_op_bswap32_i64:
  case PTC_INSTRUCTION_op_bswap64_i64: {
    Type *SwapType = nullptr;
    switch (Opcode) {
    case PTC_INSTRUCTION_op_bswap16_i32:
    case PTC_INSTRUCTION_op_bswap16_i64:
      SwapType = Builder.getInt16Ty();
      break;
    case PTC_INSTRUCTION_op_bswap32_i32:
    case PTC_INSTRUCTION_op_bswap32_i64:
      SwapType = Builder.getInt32Ty();
      break;
    case PTC_INSTRUCTION_op_bswap64_i64:
      SwapType = Builder.getInt64Ty();
      break;
    default:
      revng_unreachable("Unexpected opcode");
    }

    Value *Truncated = Builder.CreateTrunc(InArguments[0], SwapType);

    Function *BSwapFunction = Intrinsic::getDeclaration(&TheModule,
                                                        Intrinsic::bswap,
                                                        { SwapType });
    Value *Swapped = Builder.CreateCall(BSwapFunction, Truncated);

    return v{ Builder.CreateZExt(Swapped, RegisterType) };
  }
  case PTC_INSTRUCTION_op_set_label: {
    unsigned LabelId = ptc.get_arg_label_id(ConstArguments[0]);

    std::stringstream LabelSS;
    LabelSS << "bb." << JumpTargets.nameForAddress(LastPC);
    LabelSS << "_L" << std::dec << LabelId;
    std::string Label = LabelSS.str();

    BasicBlock *Fallthrough = nullptr;
    auto ExistingBasicBlock = LabeledBasicBlocks.find(Label);

    if (ExistingBasicBlock == LabeledBasicBlocks.end()) {
      Fallthrough = BasicBlock::Create(Context, Label, TheFunction);
      Fallthrough->moveAfter(Builder.GetInsertBlock());
      LabeledBasicBlocks[Label] = Fallthrough;
    } else {
      // A basic block with that label already exist
      Fallthrough = LabeledBasicBlocks[Label];

      // Ensure it's empty
      revng_assert(Fallthrough->begin() == Fallthrough->end());

      // Move it to the bottom
      Fallthrough->removeFromParent();
      TheFunction->getBasicBlockList().push_back(Fallthrough);
    }

    Builder.CreateBr(Fallthrough);

    Blocks.push_back(Fallthrough);
    Builder.SetInsertPoint(Fallthrough);
    Variables.newBasicBlock();

    return v{};
  }
  case PTC_INSTRUCTION_op_br:
  case PTC_INSTRUCTION_op_brcond_i32:
  case PTC_INSTRUCTION_op_brcond2_i32:
  case PTC_INSTRUCTION_op_brcond_i64: {
    // We take the last constant arguments, which is the LabelId both in
    // conditional and unconditional jumps
    unsigned LabelId = ptc.get_arg_label_id(ConstArguments.back());

    std::stringstream LabelSS;
    LabelSS << "bb." << JumpTargets.nameForAddress(LastPC);
    LabelSS << "_L" << std::dec << LabelId;
    std::string Label = LabelSS.str();

    BasicBlock *Fallthrough = BasicBlock::Create(Context,
                                                 Label + "_ft",
                                                 TheFunction);

    // Look for a matching label
    BasicBlock *Target = nullptr;
    auto ExistingBasicBlock = LabeledBasicBlocks.find(Label);

    // No matching label, create a temporary block
    if (ExistingBasicBlock == LabeledBasicBlocks.end()) {
      Target = BasicBlock::Create(Context, Label, TheFunction);
      LabeledBasicBlocks[Label] = Target;
    } else {
      Target = LabeledBasicBlocks[Label];
    }

    if (Opcode == PTC_INSTRUCTION_op_br) {
      // Unconditional jump
      Builder.CreateBr(Target);
    } else if (Opcode == PTC_INSTRUCTION_op_brcond_i32
               || Opcode == PTC_INSTRUCTION_op_brcond_i64) {
      // Conditional jump
      Value *Compare = CreateICmp(Builder,
                                  ConstArguments[0],
                                  InArguments[0],
                                  InArguments[1]);
      Builder.CreateCondBr(Compare, Target, Fallthrough);
    } else {
      revng_unreachable("Unhandled opcode");
    }

    Blocks.push_back(Fallthrough);
    Builder.SetInsertPoint(Fallthrough);
    Variables.newBasicBlock();

    return v{};
  }
  case PTC_INSTRUCTION_op_exit_tb: {
    auto *Zero = ConstantInt::get(Type::getInt32Ty(Context), 0);
    Builder.CreateCall(JumpTargets.exitTB(), { Zero });
    Builder.CreateUnreachable();

    ExitBlocks.push_back(Builder.GetInsertBlock());

    auto *NextBB = BasicBlock::Create(Context, "", TheFunction);
    Blocks.push_back(NextBB);
    Builder.SetInsertPoint(NextBB);
    Variables.newBasicBlock();

    return v{};
  }
  case PTC_INSTRUCTION_op_goto_tb:
    // Nothing to do here
    return v{};
  case PTC_INSTRUCTION_op_add2_i32:
  case PTC_INSTRUCTION_op_sub2_i32:
  case PTC_INSTRUCTION_op_add2_i64:
  case PTC_INSTRUCTION_op_sub2_i64: {
    Value *FirstOpLow = nullptr;
    Value *FirstOpHigh = nullptr;
    Value *SecondOpLow = nullptr;
    Value *SecondOpHigh = nullptr;

    IntegerType *DestinationType = Builder.getIntNTy(RegisterSize * 2);

    FirstOpLow = Builder.CreateZExt(InArguments[0], DestinationType);
    FirstOpHigh = Builder.CreateZExt(InArguments[1], DestinationType);
    SecondOpLow = Builder.CreateZExt(InArguments[2], DestinationType);
    SecondOpHigh = Builder.CreateZExt(InArguments[3], DestinationType);

    FirstOpHigh = Builder.CreateShl(FirstOpHigh, RegisterSize);
    SecondOpHigh = Builder.CreateShl(SecondOpHigh, RegisterSize);

    Value *FirstOp = Builder.CreateOr(FirstOpHigh, FirstOpLow);
    Value *SecondOp = Builder.CreateOr(SecondOpHigh, SecondOpLow);

    Instruction::BinaryOps BinaryOp = opcodeToBinaryOp(Opcode);

    Value *Result = Builder.CreateBinOp(BinaryOp, FirstOp, SecondOp);

    Value *ResultLow = Builder.CreateTrunc(Result, RegisterType);
    Value *ShiftedResult = Builder.CreateLShr(Result, RegisterSize);
    Value *ResultHigh = Builder.CreateTrunc(ShiftedResult, RegisterType);

    return v{ ResultLow, ResultHigh };
  }
  case PTC_INSTRUCTION_op_mulu2_i32:
  case PTC_INSTRUCTION_op_mulu2_i64:
  case PTC_INSTRUCTION_op_muls2_i32:
  case PTC_INSTRUCTION_op_muls2_i64: {
    IntegerType *DestinationType = Builder.getIntNTy(RegisterSize * 2);

    Value *FirstOp = nullptr;
    Value *SecondOp = nullptr;

    if (Opcode == PTC_INSTRUCTION_op_mulu2_i32
        || Opcode == PTC_INSTRUCTION_op_mulu2_i64) {
      FirstOp = Builder.CreateZExt(InArguments[0], DestinationType);
      SecondOp = Builder.CreateZExt(InArguments[1], DestinationType);
    } else if (Opcode == PTC_INSTRUCTION_op_muls2_i32
               || Opcode == PTC_INSTRUCTION_op_muls2_i64) {
      FirstOp = Builder.CreateSExt(InArguments[0], DestinationType);
      SecondOp = Builder.CreateSExt(InArguments[1], DestinationType);
    } else {
      revng_unreachable("Unexpected opcode");
    }

    Value *Result = Builder.CreateMul(FirstOp, SecondOp);

    Value *ResultLow = Builder.CreateTrunc(Result, RegisterType);
    Value *ShiftedResult = Builder.CreateLShr(Result, RegisterSize);
    Value *ResultHigh = Builder.CreateTrunc(ShiftedResult, RegisterType);

    return v{ ResultLow, ResultHigh };
  }
  case PTC_INSTRUCTION_op_muluh_i32:
  case PTC_INSTRUCTION_op_mulsh_i32:
  case PTC_INSTRUCTION_op_muluh_i64:
  case PTC_INSTRUCTION_op_mulsh_i64:

  case PTC_INSTRUCTION_op_setcond2_i32:

  case PTC_INSTRUCTION_op_trunc_shr_i32:
    revng_unreachable("Instruction not implemented");
  default:
    revng_unreachable("Unknown opcode");
  }
}
