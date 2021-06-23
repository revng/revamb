/// \file BinaryFile.cpp
/// \brief

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <memory>
#include <string>
#include <tuple>
#include <utility>

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/Triple.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/Object/COFF.h"
#include "llvm/Object/ELF.h"
#include "llvm/Object/MachO.h"
#include "llvm/Object/ObjectFile.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Endian.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/LEB128.h"

#include "revng/Lift/BinaryFile.h"
#include "revng/Support/CommandLine.h"
#include "revng/Support/Debug.h"

// using directives
using namespace llvm;

using std::make_pair;

using LabelList = BinaryFile::LabelList;

static Logger<> EhFrameLog("ehframe");
static Logger<> LabelsLog("labels");

const unsigned char R_MIPS_IMPLICIT_RELATIVE = 255;

namespace nooverflow {

template<typename T, typename U>
auto add(T LHS, U RHS) -> Optional<decltype(LHS + RHS)> {
  using V = decltype(LHS + RHS);
  V Result = LHS + RHS;

  if (Result < LHS)
    return {};

  return Result;
}

} // namespace nooverflow

template<typename T>
static void logAddress(T &Logger, const char *Name, MetaAddress Address) {
  if (Logger.isEnabled()) {
    Logger << Name;
    Address.dump(Logger);
    Logger << DoLog;
  }
}

template<typename T>
bool contains(const ArrayRef<T> &Container, const ArrayRef<T> &Contained) {
  return (Container.begin() <= Contained.begin()
          and Container.end() >= Contained.end());
}

template<typename R>
static void swapBytes(R &Value) {
  swapStruct(Value);
}

template<>
void swapBytes<uint32_t>(uint32_t &Value) {
  sys::swapByteOrder(Value);
}

template<typename T>
class ArrayRefReader {
private:
  ArrayRef<T> Array;
  const T *Cursor;
  bool Swap;

public:
  ArrayRefReader(ArrayRef<T> Array, bool Swap) :
    Array(Array), Cursor(Array.begin()), Swap(Swap) {}

  bool eof() const { return Cursor == Array.end(); }

  template<typename R>
  R read() {
    revng_check(Cursor + sizeof(R) > Cursor);
    revng_check(Cursor + sizeof(R) <= Array.end());

    R Result;
    memcpy(&Result, Cursor, sizeof(R));

    if (Swap)
      swapBytes<R>(Result);

    Cursor += sizeof(R);

    return Result;
  }
};

static MetaAddress
getInitialPC(Triple::ArchType Arch, bool Swap, ArrayRef<uint8_t> Command) {
  using namespace llvm::MachO;

  ArrayRefReader<uint8_t> Reader(Command, Swap);
  uint32_t Flavor = Reader.read<uint32_t>();
  uint32_t Count = Reader.read<uint32_t>();
  Optional<uint64_t> PC;

  switch (Arch) {
  case Triple::x86: {

    switch (Flavor) {
    case MachO::x86_THREAD_STATE32:
      revng_check(Count == MachO::x86_THREAD_STATE32_COUNT);
      PC = Reader.read<x86_thread_state32_t>().eip;
      break;

    case MachO::x86_THREAD_STATE:
      revng_check(Count == MachO::x86_THREAD_STATE_COUNT);
      PC = Reader.read<x86_thread_state_t>().uts.ts32.eip;
      break;

    default:
      revng_abort();
    }

    revng_check(Reader.eof());

  } break;

  case Triple::x86_64: {

    switch (Flavor) {
    case MachO::x86_THREAD_STATE64:
      revng_check(Count == MachO::x86_THREAD_STATE64_COUNT);
      PC = Reader.read<x86_thread_state64_t>().rip;
      break;

    case MachO::x86_THREAD_STATE:
      revng_check(Count == MachO::x86_THREAD_STATE_COUNT);
      PC = Reader.read<x86_thread_state_t>().uts.ts64.rip;
      break;

    default:
      revng_abort();
    }

  } break;

  case Triple::arm: {

    switch (Flavor) {
    case MachO::ARM_THREAD_STATE:
      revng_check(Count == MachO::ARM_THREAD_STATE_COUNT);
      PC = Reader.read<arm_thread_state_t>().uts.ts32.pc;
      break;

    default:
      revng_abort();
    }

  } break;

  case Triple::aarch64: {

    switch (Flavor) {
    case MachO::ARM_THREAD_STATE64:
      revng_check(Count == MachO::ARM_THREAD_STATE64_COUNT);
      PC = Reader.read<arm_thread_state64_t>().pc;
      break;

    default:
      revng_abort();
    }

  } break;

  default:
    revng_abort("Unexpected architecture for Mach-O");
    break;
  }

  revng_check(Reader.eof());

  if (PC)
    return MetaAddress::fromPC(Arch, *PC);
  else
    return MetaAddress::invalid();
}

BinaryFile::BinaryFile(std::string FilePath, uint64_t PreferedBaseAddress) :
  EntryPoint(MetaAddress::invalid()), BaseAddress(0) {
  auto BinaryOrErr = object::createBinary(FilePath);
  revng_check(BinaryOrErr, "Couldn't open the input file");
  BinaryHandle = std::move(BinaryOrErr.get());
  initialize(PreferedBaseAddress);
}

BinaryFile::BinaryFile(BinaryHandleT BinaryHandle,
                       uint64_t PreferedBaseAddress) :
  BinaryHandle(std::move(BinaryHandle)),
  EntryPoint(MetaAddress::invalid()),
  BaseAddress(0) {
  initialize(PreferedBaseAddress);
}

void BinaryFile::initialize(uint64_t PreferedBaseAddress) {
  auto *TheBinary = cast<object::ObjectFile>(BinaryHandle.getBinary());

  // TODO: QEMU should provide this information
  uint32_t InstructionAlignment = 0;
  StringRef SyscallHelper = "";
  StringRef SyscallNumberRegister = "";
  StringRef StackPointerRegister = "";
  ArrayRef<uint64_t> NoReturnSyscalls = {};
  SmallVector<ABIRegister, 20> ABIRegisters;
  uint32_t DelaySlotSize = 0;
  unsigned PCMContextIndex = ABIRegister::NotInMContext;
  llvm::StringRef WriteRegisterAsm = "";
  llvm::StringRef ReadRegisterAsm = "";
  llvm::StringRef JumpAsm = "";
  bool HasRelocationAddend;
  llvm::ArrayRef<const char> BasicBlockEndingPattern;

  using RD = RelocationDescription;
  using namespace llvm::ELF;
  using namespace model::Register;
  Architecture::RelocationTypesMap RelocationTypes;

  auto Arch = TheBinary->getArch();
  switch (Arch) {
  case Triple::x86:
    InstructionAlignment = 1;
    SyscallHelper = "helper_raise_interrupt";
    SyscallNumberRegister = "eax";
    StackPointerRegister = "esp";
    NoReturnSyscalls = {
      0xfc, // exit_group
      0x01, // exit
      0x0b // execve
    };

    HasRelocationAddend = false;

    RelocationTypes[R_386_RELATIVE] = RD(RD::BaseRelative, RD::TargetValue);
    RelocationTypes[R_386_JUMP_SLOT] = RD(RD::SymbolRelative);
    RelocationTypes[R_386_GLOB_DAT] = RD(RD::SymbolRelative);
    RelocationTypes[R_386_32] = RD(RD::SymbolRelative, RD::TargetValue);
    RelocationTypes[R_386_COPY] = RD(RD::LabelOnly, RD::TargetValue);

    ABIRegisters = { { eax_x86 }, { ebx_x86 }, { ecx_x86 }, { edx_x86 },
                     { esi_x86 }, { edi_x86 }, { ebp_x86 }, { esp_x86 } };

    BasicBlockEndingPattern = "\xcc";

    break;

  case Triple::x86_64:
    InstructionAlignment = 1;
    SyscallHelper = "helper_syscall";
    SyscallNumberRegister = "rax";
    StackPointerRegister = "rsp";
    NoReturnSyscalls = {
      0xe7, // exit_group
      0x3c, // exit
      0x3b // execve
    };
    PCMContextIndex = 0x10;

    // The offsets associated to the registers have been obtained running the
    // following command:
    //
    // scripts/compile-time-constants.py gcc ucontext.c
    //
    // where `ucontext.c` is:
    //
    // #define _GNU_SOURCE
    // #include <sys/ucontext.h>
    // #include <stdint.h>
    //
    // static ucontext_t UContext;
    //
    // #define REGISTER_OFFSET(reg) const int MContextIndex ## reg = REG_ ## reg
    //
    // REGISTER_OFFSET(R8);
    // REGISTER_OFFSET(R9);
    // REGISTER_OFFSET(R10);
    // REGISTER_OFFSET(R11);
    // REGISTER_OFFSET(R12);
    // REGISTER_OFFSET(R13);
    // REGISTER_OFFSET(R14);
    // REGISTER_OFFSET(R15);
    // REGISTER_OFFSET(RDI);
    // REGISTER_OFFSET(RSI);
    // REGISTER_OFFSET(RBP);
    // REGISTER_OFFSET(RBX);
    // REGISTER_OFFSET(RDX);
    // REGISTER_OFFSET(RAX);
    // REGISTER_OFFSET(RCX);
    // REGISTER_OFFSET(RSP);
    // REGISTER_OFFSET(RIP);

    // TODO: here we're hardcoding the offsets in the QEMU struct
    ABIRegisters = {
      { rax_x86_64, 0xD }, { rbx_x86_64, 0xB }, { rcx_x86_64, 0xE },
      { rdx_x86_64, 0xC }, { rbp_x86_64, 0xA }, { rsp_x86_64, 0xF },
      { rsi_x86_64, 0x9 }, { rdi_x86_64, 0x8 }, { r8_x86_64, 0x0 },
      { r9_x86_64, 0x1 },  { r10_x86_64, 0x2 }, { r11_x86_64, 0x3 },
      { r12_x86_64, 0x4 }, { r13_x86_64, 0x5 }, { r14_x86_64, 0x6 },
      { r15_x86_64, 0x7 }, { xmm0_x86_64 },     { xmm1_x86_64 },
      { xmm2_x86_64 },     { xmm3_x86_64 },     { xmm4_x86_64 },
      { xmm5_x86_64 },     { xmm6_x86_64 },     { xmm7_x86_64 }
    };
    WriteRegisterAsm = "movq $0, %REGISTER";
    ReadRegisterAsm = "movq %REGISTER, $0";
    JumpAsm = "jmpq *$0";

    HasRelocationAddend = true;

    RelocationTypes[R_X86_64_RELATIVE] = RD(RD::BaseRelative, RD::Addend);
    RelocationTypes[R_X86_64_JUMP_SLOT] = RD(RD::SymbolRelative);
    RelocationTypes[R_X86_64_GLOB_DAT] = RD(RD::SymbolRelative);
    RelocationTypes[R_X86_64_COPY] = RD(RD::LabelOnly, RD::TargetValue);
    // TODO: encode relocation size
    RelocationTypes[R_X86_64_32] = RD(RD::SymbolRelative, RD::Addend);
    RelocationTypes[R_X86_64_64] = RD(RD::SymbolRelative, RD::Addend);

    BasicBlockEndingPattern = "\xcc";

    break;

  case Triple::arm:
    InstructionAlignment = 4;
    SyscallHelper = "helper_exception_with_syndrome";
    SyscallNumberRegister = "r7";
    StackPointerRegister = "r13";
    NoReturnSyscalls = {
      0xf8, // exit_group
      0x1, // exit
      0xb // execve
    };
    ABIRegisters = { { r0_arm },  { r1_arm },  { r2_arm },  { r3_arm },
                     { r4_arm },  { r5_arm },  { r6_arm },  { r7_arm },
                     { r8_arm },  { r9_arm },  { r10_arm }, { r11_arm },
                     { r12_arm }, { r13_arm }, { r14_arm } };
    PCMContextIndex = 18;

    HasRelocationAddend = false;

    RelocationTypes[R_ARM_RELATIVE] = RD(RD::BaseRelative, RD::TargetValue);
    RelocationTypes[R_ARM_JUMP_SLOT] = RD(RD::SymbolRelative);
    RelocationTypes[R_ARM_GLOB_DAT] = RD(RD::SymbolRelative);
    RelocationTypes[R_ARM_COPY] = RD(RD::LabelOnly, RD::TargetValue);

    // bx lr
    BasicBlockEndingPattern = "\x1e\xff\x2f\xe1";

    break;

  case Triple::aarch64:
    HasRelocationAddend = false;
    InstructionAlignment = 4;
    SyscallHelper = "helper_exception_with_syndrome";
    SyscallNumberRegister = "x8";
    StackPointerRegister = "sp";
    NoReturnSyscalls = {
      0x5e, // exit_group
      0x5d, // exit
      0xdd // execve
    };
    ABIRegisters = {
      { x0_aarch64 },  { x1_aarch64 },  { x2_aarch64 },  { x3_aarch64 },
      { x4_aarch64 },  { x5_aarch64 },  { x6_aarch64 },  { x7_aarch64 },
      { x8_aarch64 },  { x9_aarch64 },  { x10_aarch64 }, { x11_aarch64 },
      { x12_aarch64 }, { x13_aarch64 }, { x14_aarch64 }, { x15_aarch64 },
      { x16_aarch64 }, { x17_aarch64 }, { x18_aarch64 }, { x19_aarch64 },
      { x20_aarch64 }, { x21_aarch64 }, { x22_aarch64 }, { x23_aarch64 },
      { x24_aarch64 }, { x25_aarch64 }, { x26_aarch64 }, { x27_aarch64 },
      { x28_aarch64 }, { x29_aarch64 }, { lr_aarch64 },  { sp_aarch64 }
    };
    HasRelocationAddend = false;

    // ret
    BasicBlockEndingPattern = "\xc0\x03\x5f\xd6";

    break;

  case Triple::mips:
  case Triple::mipsel:
    InstructionAlignment = 4;
    SyscallHelper = "helper_raise_exception";
    SyscallNumberRegister = "v0";
    StackPointerRegister = "sp";
    NoReturnSyscalls = {
      0x1096, // exit_group
      0xfa1, // exit
      0xfab // execve
    };
    DelaySlotSize = 1;
    ABIRegisters = { { v0_mips }, { v1_mips }, { a0_mips }, { a1_mips },
                     { a2_mips }, { a3_mips }, { s0_mips }, { s1_mips },
                     { s2_mips }, { s3_mips }, { s4_mips }, { s5_mips },
                     { s6_mips }, { s7_mips }, { gp_mips }, { sp_mips },
                     { fp_mips }, { ra_mips } };

    HasRelocationAddend = false;

    // R_MIPS_RELATIVE does not exist since the GOT has implicit base-relative
    // relocations
    RelocationTypes[R_MIPS_IMPLICIT_RELATIVE] = RD(RD::BaseRelative,
                                                   RD::TargetValue);
    RelocationTypes[R_MIPS_JUMP_SLOT] = RD(RD::SymbolRelative);
    RelocationTypes[R_MIPS_GLOB_DAT] = RD(RD::SymbolRelative);
    RelocationTypes[R_MIPS_COPY] = RD(RD::LabelOnly, RD::TargetValue);

    // jr ra
    BasicBlockEndingPattern = ((Arch == Triple::mips) ? "\x08\x00\xe0\x03" :
                                                        "\x03\xe0\x00\x08");

    break;

  case Triple::systemz:
    SyscallHelper = "helper_exception";
    SyscallNumberRegister = "r1";
    StackPointerRegister = "r15";
    InstructionAlignment = 2;
    NoReturnSyscalls = {
      0xf8, // exit_group
      0x1, // exit
      0xb, // execve
    };

    HasRelocationAddend = true;

    // TODO: investigate (R_390_RELATIVE does not exist)
    RelocationTypes[R_390_GLOB_DAT] = RD(RD::SymbolRelative);
    RelocationTypes[R_390_COPY] = RD(RD::LabelOnly, RD::TargetValue);

    ABIRegisters = {
      { r0_systemz },  { r1_systemz },  { r2_systemz },  { r3_systemz },
      { r4_systemz },  { r5_systemz },  { r6_systemz },  { r7_systemz },
      { r8_systemz },  { r9_systemz },  { r10_systemz }, { r11_systemz },
      { r12_systemz }, { r13_systemz }, { r14_systemz }, { r15_systemz },
      { f0_systemz },  { f1_systemz },  { f2_systemz },  { f3_systemz },
      { f4_systemz },  { f5_systemz },  { f6_systemz },  { f7_systemz },
      { f8_systemz },  { f9_systemz },  { f10_systemz }, { f11_systemz },
      { f12_systemz }, { f13_systemz }, { f14_systemz }, { f15_systemz }
    };

    break;

  default:
    revng_abort();
  }

  TheArchitecture = Architecture(TheBinary->getArch(),
                                 InstructionAlignment,
                                 1,
                                 TheBinary->isLittleEndian(),
                                 TheBinary->getBytesInAddress() * 8,
                                 SyscallHelper,
                                 SyscallNumberRegister,
                                 NoReturnSyscalls,
                                 DelaySlotSize,
                                 StackPointerRegister,
                                 ABIRegisters,
                                 PCMContextIndex,
                                 WriteRegisterAsm,
                                 ReadRegisterAsm,
                                 JumpAsm,
                                 HasRelocationAddend,
                                 std::move(RelocationTypes),
                                 BasicBlockEndingPattern);

  if (TheBinary->isELF()) {
    if (TheArchitecture.pointerSize() == 32) {
      if (TheArchitecture.isLittleEndian()) {
        if (TheArchitecture.hasRelocationAddend()) {
          parseELF<object::ELF32LE, true>(TheBinary, PreferedBaseAddress);
        } else {
          parseELF<object::ELF32LE, false>(TheBinary, PreferedBaseAddress);
        }
      } else {
        if (TheArchitecture.hasRelocationAddend()) {
          parseELF<object::ELF32BE, true>(TheBinary, PreferedBaseAddress);
        } else {
          parseELF<object::ELF32BE, false>(TheBinary, PreferedBaseAddress);
        }
      }
    } else if (TheArchitecture.pointerSize() == 64) {
      if (TheArchitecture.isLittleEndian()) {
        if (TheArchitecture.hasRelocationAddend()) {
          parseELF<object::ELF64LE, true>(TheBinary, PreferedBaseAddress);
        } else {
          parseELF<object::ELF64LE, false>(TheBinary, PreferedBaseAddress);
        }
      } else {
        if (TheArchitecture.hasRelocationAddend()) {
          parseELF<object::ELF64BE, true>(TheBinary, PreferedBaseAddress);
        } else {
          parseELF<object::ELF64BE, false>(TheBinary, PreferedBaseAddress);
        }
      }
    } else {
      revng_assert("Unexpect address size");
    }
  } else if (TheBinary->isCOFF()) {
    revng_assert(TheArchitecture.pointerSize() == 32
                   || TheArchitecture.pointerSize() == 64,
                 "Only 32/64-bit COFF files are supported");
    revng_assert(TheArchitecture.isLittleEndian() == true,
                 "Only Little-Endian COFF files are supported");
    // TODO handle relocations
    parseCOFF(TheBinary, PreferedBaseAddress);
  } else if (auto *MachO = dyn_cast<object::MachOObjectFile>(TheBinary)) {
    using namespace llvm::MachO;
    using namespace llvm::object;
    using LoadCommandInfo = MachOObjectFile::LoadCommandInfo;

    Triple::ArchType Arch = TheBinary->getArch();
    StringRef StringDataRef = TheBinary->getData();
    auto RawDataRef = ArrayRef<uint8_t>(StringDataRef.bytes_begin(),
                                        StringDataRef.size());
    bool MustSwap = TheArchitecture.isLittleEndian() != sys::IsLittleEndianHost;

    bool EntryPointFound = false;
    Optional<uint64_t> EntryPointOffset;
    for (const LoadCommandInfo &LCI : MachO->load_commands()) {
      switch (LCI.C.cmd) {

      case LC_SEGMENT:
        parseMachOSegment(RawDataRef, MachO->getSegmentLoadCommand(LCI));
        break;

      case LC_SEGMENT_64:
        parseMachOSegment(RawDataRef, MachO->getSegment64LoadCommand(LCI));
        break;

      case LC_UNIXTHREAD: {
        revng_check(not EntryPointFound);
        EntryPointFound = true;
        const uint8_t *Pointer = reinterpret_cast<const uint8_t *>(LCI.Ptr);
        ArrayRef<uint8_t> CommandBuffer(Pointer + sizeof(thread_command),
                                        LCI.C.cmdsize - sizeof(thread_command));
        revng_check(contains(RawDataRef, CommandBuffer));

        EntryPoint = getInitialPC(Arch, MustSwap, CommandBuffer);
      } break;

      case LC_MAIN:
        revng_check(not EntryPointFound);
        EntryPointFound = true;

        // This is an offset, delay translation to code for later
        EntryPointOffset = MachO->getEntryPointCommand(LCI).entryoff;
        break;

      case LC_FUNCTION_STARTS:
      case LC_DATA_IN_CODE:
      case LC_SYMTAB:
      case LC_DYSYMTAB:
        // TODO: very interesting
        break;
      }
    }

    if (EntryPointOffset)
      EntryPoint = virtualAddressFromOffset(*EntryPointOffset).toPC(Arch);

    const uint64_t PointerSize = TheArchitecture.pointerSize() / 8;

    Error TheError = Error::success();

    for (const MachOBindEntry &U : MachO->bindTable(TheError))
      registerBindEntry(&U, PointerSize);
    revng_check(not TheError);

    for (const MachOBindEntry &U : MachO->lazyBindTable(TheError))
      registerBindEntry(&U, PointerSize);
    revng_check(not TheError);

    // TODO: we should handle weak symbols
    for (const MachOBindEntry &U : MachO->weakBindTable(TheError))
      registerBindEntry(&U, PointerSize);
    revng_check(not TheError);

  } else {
    revng_assert("Unsupported file format.");
  }

  rebuildLabelsMap();
}

void BinaryFile::registerBindEntry(const object::MachOBindEntry *Entry,
                                   uint64_t PointerSize) {
  using namespace llvm::MachO;
  using namespace llvm::object;

  const auto Origin = LabelOrigin::DynamicRelocation;
  MetaAddress Target = MetaAddress::invalid();
  uint64_t Addend = static_cast<uint64_t>(Entry->addend());
  uint64_t Size = 0;

  switch (Entry->type()) {
  case BIND_TYPE_INVALID:
  case BIND_TYPE_POINTER:
    Target = fromGeneric(Entry->address());
    Size = PointerSize;
    break;
  case BIND_TYPE_TEXT_ABSOLUTE32:
    Target = fromPC(Entry->address());
    Size = 32 / 8;
    break;
  case BIND_TYPE_TEXT_PCREL32:
    Target = fromPC(Entry->address());
    Size = 32 / 8;
    Addend -= Target.address();
    break;
  default:
    revng_abort();
  }

  registerLabel(Label::createSymbolRelativeValue(Origin,
                                                 Target,
                                                 Size,
                                                 Entry->symbolName(),
                                                 SymbolType::Unknown,
                                                 Addend));
}

class FilePortion {
private:
  bool HasAddress;
  bool HasSize;
  uint64_t Size;
  MetaAddress Address;

public:
  FilePortion() :
    HasAddress(false),
    HasSize(false),
    Size(0),
    Address(MetaAddress::invalid()) {}

public:
  void setAddress(MetaAddress Address) {
    HasAddress = true;
    this->Address = Address;
  }

  void setSize(uint64_t Size) {
    HasSize = true;
    this->Size = Size;
  }

  MetaAddress addressAtOffset(uint64_t Offset) {
    revng_assert(HasAddress and HasSize);
    revng_assert(Offset <= Size);
    return Address + Offset;
  }

  template<typename T>
  MetaAddress addressAtIndex(uint64_t Index) {
    revng_assert(HasAddress and HasSize);
    uint64_t Offset = Index * sizeof(T);
    revng_assert(Offset <= Size);
    return Address + Offset;
  }

  bool isAvailable() const { return HasAddress; }

  bool isExact() const {
    revng_assert(HasAddress);
    return HasSize;
  }

  StringRef extractString(const std::vector<SegmentInfo> &Segments) const {
    ArrayRef<uint8_t> Data = extractData(Segments);
    const char *AsChar = reinterpret_cast<const char *>(Data.data());
    return StringRef(AsChar, Data.size());
  }

  template<typename T>
  ArrayRef<T> extractAs(const std::vector<SegmentInfo> &Segments) const {
    ArrayRef<uint8_t> Data = extractData(Segments);
    const size_t TypeSize = sizeof(T);
    revng_assert(Data.size() % TypeSize == 0);
    return ArrayRef<T>(reinterpret_cast<const T *>(Data.data()),
                       Data.size() / TypeSize);
  }

  ArrayRef<uint8_t>
  extractData(const std::vector<SegmentInfo> &Segments) const {
    revng_assert(HasAddress);

    for (const SegmentInfo &Segment : Segments) {
      if (Segment.contains(Address)) {
        uint64_t Offset = Address - Segment.StartVirtualAddress;
        uint64_t AvailableSize = Segment.size() - Offset;
        uint64_t TheSize = AvailableSize;

        if (HasSize) {
          revng_assert(AvailableSize >= Size);
          TheSize = Size;
        }

        return { ArrayRef<uint8_t>(Segment.Data.data() + Offset, TheSize) };
      }
    }

    revng_abort();
  }
};

template<typename T, bool HasAddend>
struct RelocationHelper {
  static uint64_t getAddend(llvm::object::Elf_Rel_Impl<T, HasAddend>);
};

template<typename T>
struct RelocationHelper<T, true> {
  static uint64_t getAddend(llvm::object::Elf_Rel_Impl<T, true> Relocation) {
    return Relocation.r_addend;
  }
};

template<typename T>
struct RelocationHelper<T, false> {
  static uint64_t getAddend(llvm::object::Elf_Rel_Impl<T, false>) { return 0; }
};

static bool shouldIgnoreSymbol(StringRef Name) {
  return Name == "$a" or Name == "$d";
}

static uint64_t u64(uint64_t Value) {
  return Value;
}

void BinaryFile::parseCOFF(object::ObjectFile *TheBinary, uint64_t) {
  using object::COFFObjectFile;

  auto TheCOFFOrErr = COFFObjectFile::create(TheBinary->getMemoryBufferRef());
  if (not TheCOFFOrErr) {
    logAllUnhandledErrors(TheCOFFOrErr.takeError(), errs(), "");
    revng_abort();
  }

  COFFObjectFile &TheCOFF = *TheCOFFOrErr.get();
  const object::pe32_header *PE32Header = TheCOFF.getPE32Header();

  MetaAddress ImageBase = MetaAddress::invalid();
  if (PE32Header) {
    // TODO: ImageBase should aligned to 4kb pages, should we check that?
    ImageBase = fromPC(PE32Header->ImageBase);

    EntryPoint = ImageBase + u64(PE32Header->AddressOfEntryPoint);
    ProgramHeaders.Count = PE32Header->NumberOfRvaAndSize;
    ProgramHeaders.Size = PE32Header->SizeOfHeaders;
  } else {
    const object::pe32plus_header *PE32PlusHeader = TheCOFF.getPE32PlusHeader();
    if (!PE32PlusHeader) {
      revng_assert("Invalid PE Header.\n");
      return;
    }

    // PE32+ Header
    ImageBase = fromPC(PE32PlusHeader->ImageBase);
    EntryPoint = ImageBase + u64(PE32PlusHeader->AddressOfEntryPoint);
    ProgramHeaders.Count = PE32PlusHeader->NumberOfRvaAndSize;
    ProgramHeaders.Size = PE32PlusHeader->SizeOfHeaders;
  }

  // Read sections
  for (const llvm::object::SectionRef &SecRef : TheCOFF.sections()) {
    unsigned Id = TheCOFF.getSectionID(SecRef);
    Expected<const object::coff_section *> SecOrErr = TheCOFF.getSection(Id);
    if (not SecOrErr) {
      logAllUnhandledErrors(SecOrErr.takeError(), errs(), "");
      revng_abort();
    }
    const object::coff_section *CoffRef = *SecOrErr;

    // VirtualSize might be larger than SizeOfRawData (extra data at the end of
    // the section) or viceversa (data mapped in memory but not present in
    // memory, e.g., .bss)
    uint64_t SegmentSize = std::min(CoffRef->VirtualSize,
                                    CoffRef->SizeOfRawData);

    using namespace nooverflow;
    SegmentInfo Segment;
    Segment.StartVirtualAddress = ImageBase + u64(CoffRef->VirtualAddress);
    Segment.EndVirtualAddress = Segment.StartVirtualAddress
                                + u64(CoffRef->VirtualSize);
    Segment.StartFileOffset = CoffRef->PointerToRawData;
    Segment.EndFileOffset = Segment.StartFileOffset + SegmentSize;
    Segment.IsExecutable = CoffRef->Characteristics
                           & COFF::IMAGE_SCN_MEM_EXECUTE;
    Segment.IsReadable = CoffRef->Characteristics & COFF::IMAGE_SCN_MEM_READ;
    Segment.IsWriteable = CoffRef->Characteristics & COFF::IMAGE_SCN_MEM_WRITE;

    StringRef StringDataRef = SecRef.getObject()->getData();
    auto RawDataRef = ArrayRef<uint8_t>(StringDataRef.bytes_begin(),
                                        StringDataRef.size());

    Segment.Data = ArrayRef<uint8_t>(*add(RawDataRef.begin(),
                                          CoffRef->PointerToRawData),
                                     SegmentSize);

    revng_assert(contains(RawDataRef, Segment.Data));

    Segments.push_back(Segment);
  }
}

template<typename T>
void BinaryFile::parseMachOSegment(ArrayRef<uint8_t> RawDataRef,
                                   const T &SegmentCommand) {
  using namespace llvm::MachO;
  using namespace llvm::object;
  using namespace nooverflow;

  SegmentInfo Segment;
  Segment.StartVirtualAddress = fromGeneric(SegmentCommand.vmaddr);
  Segment.EndVirtualAddress = fromGeneric(SegmentCommand.vmaddr)
                              + SegmentCommand.vmsize;
  Segment.StartFileOffset = SegmentCommand.fileoff;
  Segment.EndFileOffset = *add(SegmentCommand.fileoff, SegmentCommand.filesize);
  Segment.IsExecutable = SegmentCommand.initprot & VM_PROT_EXECUTE;
  Segment.IsReadable = SegmentCommand.initprot & VM_PROT_READ;
  Segment.IsWriteable = SegmentCommand.initprot & VM_PROT_WRITE;
  Segment.Data = ArrayRef<uint8_t>(*add(RawDataRef.begin(),
                                        SegmentCommand.fileoff),
                                   SegmentCommand.filesize);
  revng_assert(contains(RawDataRef, Segment.Data));

  Segments.push_back(Segment);
}

template<typename T, bool HasAddend>
void BinaryFile::parseELF(object::ObjectFile *TheBinary,
                          uint64_t PreferedBaseAddress) {
  // Parse the ELF file
  auto TheELFOrErr = object::ELFFile<T>::create(TheBinary->getData());
  if (not TheELFOrErr) {
    logAllUnhandledErrors(TheELFOrErr.takeError(), errs(), "");
    revng_abort();
  }
  object::ELFFile<T> &TheELF = *TheELFOrErr;

  // BaseAddress makes sense only for shared (relocatable, PIC) objects
  auto Type = TheELF.getHeader().e_type;
  if (Type == ELF::ET_DYN) {
    BaseAddress = PreferedBaseAddress;
  }

  revng_assert(Type == ELF::ET_DYN or Type == ELF::ET_EXEC,
               "rev.ng currently handles executables and "
               "dynamic libraries only.");

  // Look for static or dynamic symbols and relocations
  using ConstElf_ShdrPtr = const typename object::ELFFile<T>::Elf_Shdr *;
  using Elf_PhdrPtr = const typename object::ELFFile<T>::Elf_Phdr *;
  ConstElf_ShdrPtr SymtabShdr = nullptr;
  Elf_PhdrPtr DynamicPhdr = nullptr;
  Optional<MetaAddress> DynamicAddress;
  Optional<MetaAddress> EHFrameAddress;
  Optional<uint64_t> EHFrameSize;
  Optional<MetaAddress> EHFrameHdrAddress;

  auto Sections = TheELF.sections();
  if (not Sections) {
    logAllUnhandledErrors(std::move(Sections.takeError()), errs(), "");
  } else {
    for (auto &Section : *Sections) {
      auto NameOrErr = TheELF.getSectionName(Section);
      if (NameOrErr) {
        auto &Name = *NameOrErr;
        if (Name == ".symtab") {
          // TODO: check dedicated field in section header
          revng_assert(SymtabShdr == nullptr, "Duplicate .symtab");
          SymtabShdr = &Section;
        } else if (Name == ".eh_frame") {
          revng_assert(not EHFrameAddress, "Duplicate .eh_frame");
          EHFrameAddress = relocate(fromGeneric(Section.sh_addr));
          EHFrameSize = static_cast<uint64_t>(Section.sh_size);
        } else if (Name == ".dynamic") {
          revng_assert(not DynamicAddress, "Duplicate .dynamic");
          DynamicAddress = relocate(fromGeneric(Section.sh_addr));
        }
      }
    }
  }

  // If we found a symbol table
  if (SymtabShdr != nullptr && SymtabShdr->sh_link != 0) {
    // Obtain a reference to the string table
    auto Strtab = TheELF.getSection(SymtabShdr->sh_link);
    if (not Strtab) {
      logAllUnhandledErrors(std::move(Strtab.takeError()), errs(), "");
      revng_abort();
    }
    auto StrtabArray = TheELF.getSectionContents(**Strtab);
    if (not StrtabArray) {
      logAllUnhandledErrors(std::move(StrtabArray.takeError()), errs(), "");
      revng_abort();
    }
    StringRef StrtabContent(reinterpret_cast<const char *>(StrtabArray->data()),
                            StrtabArray->size());

    // Collect symbol names
    auto ELFSymbols = TheELF.symbols(SymtabShdr);
    if (not ELFSymbols) {
      logAllUnhandledErrors(std::move(ELFSymbols.takeError()), errs(), "");
      revng_abort();
    }
    for (auto &Symbol : *ELFSymbols) {
      auto Name = Symbol.getName(StrtabContent);
      if (not Name) {
        logAllUnhandledErrors(std::move(Name.takeError()), errs(), "");
        revng_abort();
      }

      auto SymbolType = SymbolType::fromELF(Symbol.getType());
      if (shouldIgnoreSymbol(*Name) or Symbol.st_shndx == ELF::SHN_UNDEF)
        continue;

      MetaAddress Address = MetaAddress::invalid();

      if (SymbolType == SymbolType::Code)
        Address = relocate(fromPC(Symbol.st_value));
      else
        Address = relocate(fromGeneric(Symbol.st_value));

      registerLabel(Label::createSymbol(LabelOrigin::StaticSymbol,
                                        Address,
                                        Symbol.st_size,
                                        *Name,
                                        SymbolType));
    }
  }

  const auto &ElfHeader = TheELF.getHeader();
  EntryPoint = relocate(fromPC(ElfHeader.e_entry));
  ProgramHeaders.Count = ElfHeader.e_phnum;
  ProgramHeaders.Size = ElfHeader.e_phentsize;

  // Loop over the program headers looking for PT_LOAD segments, read them out
  // and create a global variable for each one of them (writable or read-only),
  // assign them a section and output information about them in the linking info
  // CSV
  using Elf_Phdr = const typename object::ELFFile<T>::Elf_Phdr;
  using Elf_Dyn = const typename object::ELFFile<T>::Elf_Dyn;
  using Elf_Addr = const typename object::ELFFile<T>::Elf_Addr;

  auto ProgHeaders = TheELF.program_headers();
  if (not ProgHeaders) {
    logAllUnhandledErrors(std::move(ProgHeaders.takeError()), errs(), "");
    revng_abort();
  }

  auto RawDataRef = ArrayRef<uint8_t>(TheELF.base(), TheELF.getBufSize());

  for (Elf_Phdr &ProgramHeader : *ProgHeaders) {
    switch (ProgramHeader.p_type) {
    case ELF::PT_LOAD: {
      using namespace nooverflow;
      SegmentInfo Segment;
      auto Start = relocate(fromGeneric(ProgramHeader.p_vaddr));
      Segment.StartVirtualAddress = Start;
      Segment.EndVirtualAddress = Start + u64(ProgramHeader.p_memsz);
      Segment.StartFileOffset = ProgramHeader.p_offset;
      Segment.EndFileOffset = *add(ProgramHeader.p_offset,
                                   ProgramHeader.p_filesz);
      Segment.IsReadable = ProgramHeader.p_flags & ELF::PF_R;
      Segment.IsWriteable = ProgramHeader.p_flags & ELF::PF_W;
      Segment.IsExecutable = ProgramHeader.p_flags & ELF::PF_X;
      Segment.Data = ArrayRef<uint8_t>(*add(RawDataRef.begin(),
                                            ProgramHeader.p_offset),
                                       ProgramHeader.p_filesz);

      revng_assert(contains(RawDataRef, Segment.Data));

      // If it's an executable segment, and we've been asked so, register
      // which sections actually contain code
      if (Sections and not IgnoreDebugSymbols and Segment.IsExecutable) {
        using Elf_Shdr = const typename object::ELFFile<T>::Elf_Shdr;
        auto Inserter = std::back_inserter(Segment.ExecutableSections);
        for (Elf_Shdr &SectionHeader : *Sections) {
          if (SectionHeader.sh_flags & ELF::SHF_EXECINSTR) {
            auto SectionStart = relocate(fromGeneric(SectionHeader.sh_addr));
            auto SectionEnd = SectionStart + u64(SectionHeader.sh_size);
            Inserter = make_pair(SectionStart, SectionEnd);
          }
        }
      }

      Segments.push_back(Segment);

      // Check if it's the segment containing the program headers
      auto ProgramHeaderStart = ProgramHeader.p_offset;
      auto ProgramHeaderEnd = ProgramHeader.p_offset
                              + u64(ProgramHeader.p_filesz);
      if (ProgramHeaderStart <= ElfHeader.e_phoff
          && ElfHeader.e_phoff < ProgramHeaderEnd) {
        MetaAddress PhdrAddress = (relocate(fromGeneric(ProgramHeader.p_vaddr))
                                   + u64(ElfHeader.e_phoff)
                                   - u64(ProgramHeader.p_offset));
        ProgramHeaders.Address = PhdrAddress;
      }
    } break;

    case ELF::PT_GNU_EH_FRAME:
      revng_assert(!EHFrameHdrAddress);
      EHFrameHdrAddress = relocate(fromGeneric(ProgramHeader.p_vaddr));
      break;

    case ELF::PT_DYNAMIC:
      revng_assert(DynamicPhdr == nullptr, "Duplicate .dynamic program header");
      DynamicPhdr = &ProgramHeader;
      MetaAddress DynamicPhdrMA = relocate(fromGeneric(DynamicPhdr->p_vaddr));
      revng_assert(not DynamicAddress or DynamicPhdrMA == *DynamicAddress,
                   ".dynamic and PT_DYNAMIC have different addresses");
      DynamicAddress = relocate(DynamicPhdrMA);
      break;
    }
  }

  revng_assert((DynamicPhdr != nullptr) == (DynamicAddress.hasValue()));

  Optional<uint64_t> FDEsCount;
  if (EHFrameHdrAddress) {
    MetaAddress Address = MetaAddress::invalid();

    std::tie(Address, FDEsCount) = ehFrameFromEhFrameHdr<T>(*EHFrameHdrAddress);
    if (EHFrameAddress) {
      revng_assert(*EHFrameAddress == Address);
    }

    EHFrameAddress = Address;
  }

  if (EHFrameAddress)
    parseEHFrame<T>(*EHFrameAddress, FDEsCount, EHFrameSize);

  // Parse the .dynamic table
  if (DynamicPhdr != nullptr) {
    SmallVector<uint64_t, 10> NeededLibraryNameOffsets;

    FilePortion DynstrPortion;
    FilePortion DynsymPortion;
    FilePortion ReldynPortion;
    FilePortion RelpltPortion;
    FilePortion GotPortion;
    Optional<uint64_t> SymbolsCount;
    Optional<uint64_t> MIPSFirstGotSymbol;
    Optional<uint64_t> MIPSLocalGotEntries;
    bool IsMIPS = (TheArchitecture.type() == Triple::mips
                   or TheArchitecture.type() == Triple::mipsel);

    auto DynamicEntries = TheELF.dynamicEntries();
    if (not DynamicEntries) {
      logAllUnhandledErrors(std::move(DynamicEntries.takeError()), errs(), "");
      revng_abort();
    }
    for (Elf_Dyn &DynamicTag : *DynamicEntries) {

      auto TheTag = DynamicTag.getTag();
      MetaAddress Relocated = relocate(fromGeneric(DynamicTag.getPtr()));
      switch (TheTag) {
      case ELF::DT_NEEDED:
        NeededLibraryNameOffsets.push_back(DynamicTag.getVal());
        break;

      case ELF::DT_STRTAB:
        DynstrPortion.setAddress(Relocated);
        break;

      case ELF::DT_STRSZ:
        DynstrPortion.setSize(DynamicTag.getVal());
        break;

      case ELF::DT_SYMTAB:
        DynsymPortion.setAddress(Relocated);
        break;

      case ELF::DT_JMPREL:
        RelpltPortion.setAddress(Relocated);
        break;

      case ELF::DT_PLTRELSZ:
        RelpltPortion.setSize(DynamicTag.getVal());
        break;

      case ELF::DT_REL:
      case ELF::DT_RELA:
        revng_assert(TheTag == (HasAddend ? ELF::DT_RELA : ELF::DT_REL));
        ReldynPortion.setAddress(Relocated);
        break;

      case ELF::DT_RELSZ:
      case ELF::DT_RELASZ:
        revng_assert(TheTag == (HasAddend ? ELF::DT_RELASZ : ELF::DT_RELSZ));
        ReldynPortion.setSize(DynamicTag.getVal());
        break;

      case ELF::DT_PLTGOT:
        GotPortion.setAddress(Relocated);

        // Obtaint the canonical value of the global pointer in MIPS
        if (IsMIPS)
          CanonicalValues["gp"] = (Relocated + 0x7ff0).address();
        break;

      case ELF::DT_MIPS_SYMTABNO:
        if (IsMIPS)
          SymbolsCount = DynamicTag.getVal();
        break;

      case ELF::DT_MIPS_GOTSYM:
        if (IsMIPS)
          MIPSFirstGotSymbol = DynamicTag.getVal();
        break;

      case ELF::DT_MIPS_LOCAL_GOTNO:
        if (IsMIPS)
          MIPSLocalGotEntries = DynamicTag.getVal();
        break;
      }
    }

    if (NeededLibraryNames.size() > 0)
      revng_assert(DynstrPortion.isAvailable());

    // In MIPS the GOT has one entry per symbol
    if (IsMIPS and SymbolsCount and MIPSFirstGotSymbol
        and MIPSLocalGotEntries) {
      uint32_t GotEntries = (*MIPSLocalGotEntries
                             + (*SymbolsCount - *MIPSFirstGotSymbol));
      GotPortion.setSize(GotEntries * sizeof(Elf_Addr));
    }

    StringRef Dynstr;

    if (DynstrPortion.isAvailable()) {
      Dynstr = DynstrPortion.extractString(Segments);
      for (auto Offset : NeededLibraryNameOffsets) {
        StringRef LibraryName = Dynstr.slice(Offset, Dynstr.size());
        NeededLibraryNames.push_back(LibraryName.data());
      }
    }

    // Collect symbols count and code pointers in image base-relative
    // relocations

    if (not SymbolsCount) {
      SymbolsCount = std::max(symbolsCount<T, HasAddend>(ReldynPortion),
                              symbolsCount<T, HasAddend>(RelpltPortion));
    }

    // Collect function addresses contained in dynamic symbols
    if (SymbolsCount and *SymbolsCount > 0 and DynsymPortion.isAvailable()) {
      using Elf_Sym = llvm::object::Elf_Sym_Impl<T>;
      DynsymPortion.setSize(*SymbolsCount * sizeof(Elf_Sym));
      ArrayRef<Elf_Sym> Symbols = DynsymPortion.extractAs<Elf_Sym>(Segments);
      for (Elf_Sym Symbol : Symbols) {
        auto Name = Symbol.getName(Dynstr);
        if (not Name) {
          logAllUnhandledErrors(std::move(Name.takeError()), errs(), "");
          revng_abort();
        }

        auto SymbolType = SymbolType::fromELF(Symbol.getType());

        if (shouldIgnoreSymbol(*Name) or Symbol.st_shndx == ELF::SHN_UNDEF)
          continue;

        MetaAddress Address = MetaAddress::invalid();

        if (SymbolType == SymbolType::Code)
          Address = relocate(fromPC(Symbol.st_value));
        else
          Address = relocate(fromGeneric(Symbol.st_value));

        registerLabel(Label::createSymbol(LabelOrigin::DynamicSymbol,
                                          Address,
                                          Symbol.st_size,
                                          *Name,
                                          SymbolType));
      }

      using Elf_Rel = llvm::object::Elf_Rel_Impl<T, HasAddend>;
      if (ReldynPortion.isAvailable()) {
        auto Relocations = ReldynPortion.extractAs<Elf_Rel>(Segments);
        registerRelocations<T, HasAddend>(Relocations,
                                          DynsymPortion,
                                          DynstrPortion);
      }

      if (RelpltPortion.isAvailable()) {
        auto Relocations = RelpltPortion.extractAs<Elf_Rel>(Segments);
        registerRelocations<T, HasAddend>(Relocations,
                                          DynsymPortion,
                                          DynstrPortion);
      }

      if (IsMIPS and GotPortion.isAvailable()) {
        std::vector<Elf_Rel> MIPSImplicitRelocations;
        uint32_t GotIndex = 0;

        // Perform local relocations on GOT
        if (MIPSLocalGotEntries) {
          for (; GotIndex < *MIPSLocalGotEntries; GotIndex++) {
            auto Address = GotPortion.addressAtIndex<Elf_Addr>(GotIndex);
            Elf_Rel NewRelocation;
            NewRelocation.r_offset = Address.address();
            NewRelocation.setSymbolAndType(0, R_MIPS_IMPLICIT_RELATIVE, false);
            MIPSImplicitRelocations.push_back(NewRelocation);
          }
        }

        // Relocate the remaining entries of the GOT with global symbols
        if (MIPSFirstGotSymbol and SymbolsCount and DynstrPortion.isAvailable()
            and DynsymPortion.isAvailable()) {
          for (uint32_t SymbolIndex = *MIPSFirstGotSymbol;
               SymbolIndex < *SymbolsCount;
               SymbolIndex++, GotIndex++) {
            auto Address = GotPortion.addressAtIndex<Elf_Addr>(GotIndex);

            Elf_Rel NewRelocation;
            NewRelocation.r_offset = Address.address();
            NewRelocation.setSymbolAndType(SymbolIndex,
                                           llvm::ELF::R_MIPS_JUMP_SLOT,
                                           false);
            MIPSImplicitRelocations.push_back(NewRelocation);
          }
        }

        auto Relocations = ArrayRef<Elf_Rel>(MIPSImplicitRelocations);
        registerRelocations<T, HasAddend>(Relocations,
                                          DynsymPortion,
                                          DynstrPortion);
      }
    }
  }

  for (Label &L : Labels) {
    MetaAddress MA = MetaAddress::invalid();

    if (L.isSymbol() and L.isCode())
      MA = relocate(L.address());
    else if (L.isBaseRelativeValue())
      MA = relocate(fromPC(L.value()));

    if (MA.isValid())
      CodePointers.insert(MA);
  }
}

template<typename T, bool HasAddend>
uint64_t BinaryFile::symbolsCount(const FilePortion &Relocations) {
  using Elf_Rel = llvm::object::Elf_Rel_Impl<T, HasAddend>;

  if (not Relocations.isAvailable())
    return 0;

  uint32_t SymbolsCount = 0;
  revng_assert(Relocations.isExact());
  for (Elf_Rel Relocation : Relocations.extractAs<Elf_Rel>(Segments))
    SymbolsCount = std::max(SymbolsCount, Relocation.getSymbol(false) + 1);

  return SymbolsCount;
}

Optional<uint64_t> BinaryFile::readRawValue(MetaAddress Address,
                                            unsigned Size,
                                            Endianess E) const {
  bool IsLittleEndian = ((E == OriginalEndianess) ?
                           architecture().isLittleEndian() :
                           E == LittleEndian);

  for (auto &Segment : segments()) {
    // Note: we also consider writeable memory areas because, despite being
    // modifiable, can contain useful information
    if (Segment.contains(Address, Size) && Segment.IsReadable) {
      uint64_t Offset = Address - Segment.StartVirtualAddress;
      // Handle the [p_filesz, p_memsz] portion of the segment
      if (Offset > Segment.Data.size())
        return 0;

      const unsigned char *Start = Segment.Data.data() + Offset;

      char Buffer[8] = { 0 };
      memcpy(&Buffer,
             Start,
             std::min(static_cast<size_t>(Size), Segment.Data.size() - Offset));

      using support::endianness;
      using support::endian::read;
      switch (Size) {
      case 1:
        return read<uint8_t, endianness::little, 1>(&Buffer);
      case 2:
        if (IsLittleEndian)
          return read<uint16_t, endianness::little, 1>(&Buffer);
        else
          return read<uint16_t, endianness::big, 1>(&Buffer);
      case 4:
        if (IsLittleEndian)
          return read<uint32_t, endianness::little, 1>(&Buffer);
        else
          return read<uint32_t, endianness::big, 1>(&Buffer);
      case 8:
        if (IsLittleEndian)
          return read<uint64_t, endianness::little, 1>(&Buffer);
        else
          return read<uint64_t, endianness::big, 1>(&Buffer);
      default:
        revng_abort("Unexpected read size");
      }
    }
  }

  return Optional<uint64_t>();
}

Label BinaryFile::parseRelocation(unsigned char RelocationType,
                                  MetaAddress Target,
                                  uint64_t Addend,
                                  StringRef SymbolName,
                                  uint64_t SymbolSize,
                                  SymbolType::Values SymbolType) {

  const auto &RelocationTypes = TheArchitecture.relocationTypes();
  auto It = RelocationTypes.find(RelocationType);
  if (It == RelocationTypes.end()) {
    dbg << "Warning: unhandled relocation type "
        << static_cast<int>(RelocationType) << "\n";
    return Label::createInvalid();
  }

  uint64_t Offset;

  using RD = RelocationDescription;
  const RD &Description = It->second;
  uint64_t PointerSize = TheArchitecture.pointerSize() / 8;

  switch (Description.Offset) {
  case RD::None:
    Offset = 0;
    break;

  case RD::Addend:
    Offset = Addend;
    break;

  case RD::TargetValue:
    Optional<uint64_t> ReadResult = readRawValue(Target, PointerSize);
    if (not ReadResult)
      return Label::createInvalid();
    Offset = *ReadResult;
    break;
  }

  const auto Origin = LabelOrigin::DynamicRelocation;
  switch (Description.Type) {
  case RD::BaseRelative:
    return Label::createBaseRelativeValue(Origin, Target, PointerSize, Offset);

  case RD::LabelOnly:
    if (shouldIgnoreSymbol(SymbolName))
      return Label::createInvalid();
    return Label::createSymbol(Origin,
                               Target,
                               SymbolSize,
                               SymbolName,
                               SymbolType);

  case RD::SymbolRelative:
    if (shouldIgnoreSymbol(SymbolName))
      return Label::createInvalid();
    return Label::createSymbolRelativeValue(Origin,
                                            Target,
                                            PointerSize,
                                            SymbolName,
                                            SymbolType,
                                            Offset);

  case RD::Invalid:
    revng_abort("Invalid relocation type");
    break;
  }

  revng_abort();
}

template<typename T, bool HasAddend>
void BinaryFile::registerRelocations(Elf_Rel_Array<T, HasAddend> Relocations,
                                     const FilePortion &Dynsym,
                                     const FilePortion &Dynstr) {
  using Elf_Rel = llvm::object::Elf_Rel_Impl<T, HasAddend>;
  using Elf_Sym = llvm::object::Elf_Sym_Impl<T>;

  ArrayRef<Elf_Sym> Symbols;
  if (Dynsym.isAvailable())
    Symbols = Dynsym.extractAs<Elf_Sym>(Segments);

  for (Elf_Rel Relocation : Relocations) {
    auto Type = static_cast<unsigned char>(Relocation.getType(false));
    uint64_t Addend = RelocationHelper<T, HasAddend>::getAddend(Relocation);
    MetaAddress Address = relocate(fromGeneric(Relocation.r_offset));

    StringRef SymbolName;
    uint64_t SymbolSize = 0;
    unsigned char SymbolType = llvm::ELF::STT_NOTYPE;
    if (Dynsym.isAvailable() and Dynstr.isAvailable()) {
      uint32_t SymbolIndex = Relocation.getSymbol(false);
      revng_check(SymbolIndex < Symbols.size());
      const Elf_Sym &Symbol = Symbols[SymbolIndex];
      auto Result = Symbol.getName(Dynstr.extractString(Segments));
      if (Result)
        SymbolName = *Result;
      SymbolSize = Symbol.st_size;
      SymbolType = Symbol.getType();
    }

    registerLabel(parseRelocation(Type,
                                  Address,
                                  Addend,
                                  SymbolName,
                                  SymbolSize,
                                  SymbolType::fromELF(SymbolType)));
  }
}

static LabelList &operator+=(LabelList &This, const LabelList &Other) {
  This.insert(std::end(This), std::begin(Other), std::end(Other));
  return This;
}

void BinaryFile::rebuildLabelsMap() {
  using Interval = boost::icl::interval<MetaAddress, compareAddress>;

  // Clear the map
  LabelsMap.clear();

  // Identify all the 0-sized labels
  std::vector<Label *> ZeroSizedLabels;
  for (Label &L : Labels)
    if (L.isSymbol() and L.size() == 0)
      ZeroSizedLabels.push_back(&L);

  // Sort the 0-sized labels
  auto Compare = [](Label *This, Label *Other) {
    return This->address().addressLowerThan(Other->address());
  };
  std::sort(ZeroSizedLabels.begin(), ZeroSizedLabels.end(), Compare);

  // Create virtual terminator label
  revng_assert(Segments.size() > 0);
  MetaAddress HighestAddress = Segments[0].EndVirtualAddress;
  for (const SegmentInfo &Segment : skip(1, Segments))
    if (Segment.EndVirtualAddress.addressGreaterThan(HighestAddress))
      HighestAddress = Segment.EndVirtualAddress;

  Label EndLabel = Label::createSymbol(LabelOrigin::Unknown,
                                       HighestAddress,
                                       0,
                                       "",
                                       SymbolType::Unknown);
  ZeroSizedLabels.push_back(&EndLabel);

  // Insert the 0-sized labels in the map
  for (unsigned I = 0; I < ZeroSizedLabels.size() - 1; I++) {
    MetaAddress Start = ZeroSizedLabels[I]->address();

    const SegmentInfo *Segment = findSegment(Start);
    if (Segment == nullptr)
      continue;

    // Limit the symbol to the end of the segment containing it
    MetaAddress End;
    MetaAddress NextAddress = ZeroSizedLabels[I + 1]->address();
    MetaAddress LastAddress = Segment->EndVirtualAddress;
    if (NextAddress.addressLowerThan(LastAddress)) {
      End = NextAddress;
    } else {
      End = LastAddress;
    }

    revng_assert(Start.addressLowerThanOrEqual(End));

    // Register virtual size
    ZeroSizedLabels[I]->setVirtualSize(End - Start);
  }

  // Insert all the other labels in the map
  for (Label &L : Labels) {
    MetaAddress Start = L.address();
    MetaAddress End = L.address() + L.size();
    LabelsMap += make_pair(Interval::right_open(Start, End), LabelList{ &L });
  }

  // Dump the map out
  if (LabelsLog.isEnabled()) {
    for (auto &P : LabelsMap) {
      dbg << "[";
      P.first.lower().dump(dbg);
      dbg << ",";
      P.first.upper().dump(dbg);
      dbg << "]\n";
      for (const Label *L : P.second) {
        dbg << "  ";
        L->dump(dbg);
        dbg << "\n";
      }
      dbg << "\n";
    }
  }
}

//
// .eh_frame-related functions
//
template<typename E>
class DwarfReader {
public:
  DwarfReader(Triple::ArchType Architecture,
              ArrayRef<uint8_t> Buffer,
              MetaAddress Address) :
    Architecture(Architecture),
    Address(Address),
    Start(Buffer.data()),
    Cursor(Buffer.data()),
    End(Buffer.data() + Buffer.size()) {}

  uint8_t readNextU8() { return readNext<uint8_t>(); }
  uint16_t readNextU16() { return readNext<uint16_t>(); }
  uint32_t readNextU32() { return readNext<uint32_t>(); }
  uint64_t readNextU64() { return readNext<uint64_t>(); }
  uint64_t readNextU() {
    if (is64())
      return readNextU64();
    else
      return readNextU32();
  }

  uint64_t readULEB128() {
    unsigned Length;
    uint64_t Result = decodeULEB128(Cursor, &Length);
    Cursor += Length;
    revng_assert(Cursor <= End);
    return Result;
  }

  int64_t readSLEB128() {
    unsigned Length;
    int64_t Result = decodeSLEB128(Cursor, &Length);
    Cursor += Length;
    revng_assert(Cursor <= End);
    return Result;
  }

  int64_t readSignedValue(unsigned Encoding) {
    return static_cast<int64_t>(readValue(Encoding));
  }

  uint64_t readUnsignedValue(unsigned Encoding) {
    return static_cast<uint64_t>(readValue(Encoding));
  }

  Pointer
  readPointer(unsigned Encoding, MetaAddress Base = MetaAddress::invalid()) {
    revng_assert((Encoding & ~(0x70 | 0x0F | dwarf::DW_EH_PE_indirect)) == 0);

    // Handle PC-relative values
    revng_assert(Cursor >= Start);
    if ((Encoding & 0x70) == dwarf::DW_EH_PE_pcrel) {
      revng_assert(Base.isInvalid());
      Base = Address + (Cursor - Start);
    }

    if (isSigned(Encoding & 0x0F)) {
      return readPointerInternal(readSignedValue(Encoding), Encoding, Base);
    } else {
      return readPointerInternal(readUnsignedValue(Encoding), Encoding, Base);
    }
  }

  void moveTo(uint64_t Offset) {
    const uint8_t *NewCursor = Start + Offset;
    revng_assert(NewCursor >= Cursor && NewCursor <= End);
    Cursor = NewCursor;
  }

  bool eof() const { return Cursor >= End; }
  uint64_t offset() const { return Cursor - Start; }

private:
  template<typename T>
  std::conditional_t<std::numeric_limits<T>::is_signed, int64_t, uint64_t>
  readNext() {
    constexpr bool IsSigned = std::numeric_limits<T>::is_signed;
    using ReturnType = std::conditional_t<IsSigned, int64_t, uint64_t>;
    revng_assert(Cursor + sizeof(T) <= End);
    auto Result = static_cast<T>(Endianess<T, E>::read(Cursor));
    Cursor += sizeof(T);
    return static_cast<ReturnType>(Result);
  }

  static bool isSigned(unsigned Format) {
    switch (Format) {
    case dwarf::DW_EH_PE_sleb128:
    case dwarf::DW_EH_PE_signed:
    case dwarf::DW_EH_PE_sdata2:
    case dwarf::DW_EH_PE_sdata4:
    case dwarf::DW_EH_PE_sdata8:
      return true;
    case dwarf::DW_EH_PE_absptr:
    case dwarf::DW_EH_PE_uleb128:
    case dwarf::DW_EH_PE_udata2:
    case dwarf::DW_EH_PE_udata4:
    case dwarf::DW_EH_PE_udata8:
      return false;
    default:
      revng_abort("Unknown Encoding");
    }
  }

  uint64_t readValue(unsigned Encoding) {
    revng_assert((Encoding & ~(0x70 | 0x0F | dwarf::DW_EH_PE_indirect)) == 0);

    // Extract the format
    unsigned Format = Encoding & 0x0F;
    switch (Format) {
    case dwarf::DW_EH_PE_uleb128:
      return readULEB128();
    case dwarf::DW_EH_PE_sleb128:
      return readSLEB128();
    case dwarf::DW_EH_PE_absptr:
      if (is64())
        return readNext<uint64_t>();
      else
        return readNext<uint32_t>();
    case dwarf::DW_EH_PE_signed:
      if (is64())
        return readNext<int64_t>();
      else
        return readNext<int32_t>();
    case dwarf::DW_EH_PE_udata2:
      return readNext<uint16_t>();
    case dwarf::DW_EH_PE_sdata2:
      return readNext<int16_t>();
    case dwarf::DW_EH_PE_udata4:
      return readNext<uint32_t>();
    case dwarf::DW_EH_PE_sdata4:
      return readNext<int32_t>();
    case dwarf::DW_EH_PE_udata8:
      return readNext<uint64_t>();
    case dwarf::DW_EH_PE_sdata8:
      return readNext<int64_t>();
    default:
      revng_unreachable("Unknown Encoding");
    }
  }

  template<typename T>
  Pointer readPointerInternal(T Value, unsigned Encoding, MetaAddress Base) {
    bool IsIndirect = Encoding & dwarf::DW_EH_PE_indirect;

    if (Base.isInvalid()) {
      return Pointer(IsIndirect, MetaAddress::fromGeneric(Architecture, Value));
    } else {
      unsigned EncodingRelative = Encoding & 0x70;
      revng_assert(EncodingRelative == 0 || EncodingRelative == 0x10);
      return Pointer(IsIndirect, Base + Value);
    }
  }

  bool is64() const;

private:
  Triple::ArchType Architecture;
  MetaAddress Address;
  const uint8_t *Start;
  const uint8_t *Cursor;
  const uint8_t *End;
};

template<>
bool DwarfReader<object::ELF32BE>::is64() const {
  return false;
}
template<>
bool DwarfReader<object::ELF32LE>::is64() const {
  return false;
}
template<>
bool DwarfReader<object::ELF64BE>::is64() const {
  return true;
}
template<>
bool DwarfReader<object::ELF64LE>::is64() const {
  return true;
}

template<typename T>
std::pair<MetaAddress, uint64_t>
BinaryFile::ehFrameFromEhFrameHdr(MetaAddress EHFrameHdrAddress) {
  auto R = getAddressData(EHFrameHdrAddress);
  revng_assert(R, ".eh_frame_hdr section not available in any segment");
  llvm::ArrayRef<uint8_t> EHFrameHdr = *R;

  DwarfReader<T> EHFrameHdrReader(TheArchitecture.type(),
                                  EHFrameHdr,
                                  EHFrameHdrAddress);

  uint64_t VersionNumber = EHFrameHdrReader.readNextU8();
  revng_assert(VersionNumber == 1);

  // ExceptionFrameEncoding
  uint64_t ExceptionFrameEncoding = EHFrameHdrReader.readNextU8();

  // FDEsCountEncoding
  unsigned FDEsCountEncoding = EHFrameHdrReader.readNextU8();

  // LookupTableEncoding
  EHFrameHdrReader.readNextU8();

  Pointer EHFramePointer = EHFrameHdrReader.readPointer(ExceptionFrameEncoding);
  uint64_t FDEsCount = EHFrameHdrReader.readUnsignedValue(FDEsCountEncoding);

  return { getGenericPointer<T>(EHFramePointer), FDEsCount };
}

template<typename T>
void BinaryFile::parseEHFrame(MetaAddress EHFrameAddress,
                              Optional<uint64_t> FDEsCount,
                              Optional<uint64_t> EHFrameSize) {
  revng_assert(FDEsCount || EHFrameSize);

  auto R = getAddressData(EHFrameAddress);

  // Sometimes the .eh_frame section is present but not mapped in memory. This
  // means it cannot be used at runtime, therefore we can ignore it.
  if (!R)
    return;
  llvm::ArrayRef<uint8_t> EHFrame = *R;

  DwarfReader<T> EHFrameReader(TheArchitecture.type(), EHFrame, EHFrameAddress);

  // A few fields of the CIE are used when decoding the FDE's.  This struct
  // will cache those fields we need so that we don't have to decode it
  // repeatedly for each FDE that references it.
  struct DecodedCIE {
    Optional<uint32_t> FDEPointerEncoding;
    Optional<uint32_t> LSDAPointerEncoding;
    bool hasAugmentationLength;
  };

  // Map from the start offset of the CIE to the cached data for that CIE.
  DenseMap<uint64_t, DecodedCIE> CachedCIEs;
  unsigned FDEIndex = 0;

  while (!EHFrameReader.eof()
         && ((FDEsCount && FDEIndex < *FDEsCount)
             || (EHFrameSize && EHFrameReader.offset() < *EHFrameSize))) {

    uint64_t StartOffset = EHFrameReader.offset();

    // Read the length of the entry
    uint64_t Length = EHFrameReader.readNextU32();
    if (Length == 0xffffffff)
      Length = EHFrameReader.readNextU64();

    // Compute the end offset of the entry
    uint64_t OffsetAfterLength = EHFrameReader.offset();
    uint64_t EndOffset = OffsetAfterLength + Length;

    // Zero-sized entry, skip it
    if (Length == 0) {
      revng_assert(EHFrameReader.offset() == EndOffset);
      continue;
    }

    // Get the entry ID, 0 means it's a CIE, otherwise it's a FDE
    uint32_t ID = EHFrameReader.readNextU32();
    if (ID == 0) {
      // This is a CIE
      revng_log(EhFrameLog, "New CIE");

      // Ensure the version is the one we expect
      uint32_t Version = EHFrameReader.readNextU8();
      revng_assert(Version == 1);

      // Parse a null terminated augmentation string
      SmallString<8> AugmentationString;
      for (uint8_t Char = EHFrameReader.readNextU8(); Char != 0;
           Char = EHFrameReader.readNextU8())
        AugmentationString.push_back(Char);

      // Optionally parse the EH data if the augmentation string says it's
      // there
      if (StringRef(AugmentationString).count("eh") != 0)
        EHFrameReader.readNextU();

      // CodeAlignmentFactor
      EHFrameReader.readULEB128();

      // DataAlignmentFactor
      EHFrameReader.readULEB128();

      // ReturnAddressRegister
      EHFrameReader.readNextU8();

      Optional<uint64_t> AugmentationLength;
      Optional<uint32_t> LSDAPointerEncoding;
      Optional<uint32_t> PersonalityEncoding;
      Optional<uint32_t> FDEPointerEncoding;
      if (!AugmentationString.empty() && AugmentationString.front() == 'z') {
        AugmentationLength = EHFrameReader.readULEB128();

        // Walk the augmentation string to get all the augmentation data.
        for (unsigned I = 1, e = AugmentationString.size(); I != e; ++I) {
          char Char = AugmentationString[I];
          switch (Char) {
          case 'e':
            revng_assert((I + 1) != e && AugmentationString[I + 1] == 'h',
                         "Expected 'eh' in augmentation string");
            break;
          case 'L':
            // This is the only information we really care about, all the rest
            // is processed just so we can get here
            revng_assert(!LSDAPointerEncoding, "Duplicate LSDA encoding");
            LSDAPointerEncoding = EHFrameReader.readNextU8();
            break;
          case 'P': {
            revng_assert(!PersonalityEncoding, "Duplicate personality");
            PersonalityEncoding = EHFrameReader.readNextU8();
            // Personality
            Pointer Personality;
            Personality = EHFrameReader.readPointer(*PersonalityEncoding);
            auto PersonalityPtr = getCodePointer<T>(Personality);
            logAddress(EhFrameLog, "Personality function: ", PersonalityPtr);

            // TODO: technically this is not a landing pad
            LandingPads.insert(PersonalityPtr);
            break;
          }
          case 'R':
            revng_assert(!FDEPointerEncoding, "Duplicate FDE encoding");
            FDEPointerEncoding = EHFrameReader.readNextU8();
            break;
          case 'z':
            revng_unreachable("'z' must be first in the augmentation string");
          }
        }
      }

      // Cache this entry
      CachedCIEs[StartOffset] = { FDEPointerEncoding,
                                  LSDAPointerEncoding,
                                  AugmentationLength.hasValue() };

    } else {
      // This is an FDE
      FDEIndex++;

      // The CIE pointer for an FDE is the same location as the ID which we
      // already read
      uint64_t CIEOffset = OffsetAfterLength - ID;

      // Ensure we already met this CIE
      auto CIEIt = CachedCIEs.find(CIEOffset);
      revng_assert(CIEIt != CachedCIEs.end(),
                   "Couldn't find CIE at offset in to __eh_frame section");

      // Ensure we have at least the pointer encoding
      const DecodedCIE &CIE = CIEIt->getSecond();
      revng_assert(CIE.FDEPointerEncoding,
                   "FDE references CIE which did not set pointer encoding");

      // PCBegin
      auto PCBeginPointer = EHFrameReader.readPointer(*CIE.FDEPointerEncoding);
      MetaAddress PCBegin = getGenericPointer<T>(PCBeginPointer);
      logAddress(EhFrameLog, "PCBegin: ", PCBegin);

      // PCRange
      EHFrameReader.readPointer(*CIE.FDEPointerEncoding);

      if (CIE.hasAugmentationLength)
        EHFrameReader.readULEB128();

      // Decode the LSDA if the CIE augmentation string said we should.
      if (CIE.LSDAPointerEncoding) {
        auto LSDAPointer = EHFrameReader.readPointer(*CIE.LSDAPointerEncoding);
        parseLSDA<T>(PCBegin, getGenericPointer<T>(LSDAPointer));
      }
    }

    // Skip all the remaining parts
    EHFrameReader.moveTo(EndOffset);
  }
}

template<typename T>
void BinaryFile::parseLSDA(MetaAddress FDEStart, MetaAddress LSDAAddress) {
  logAddress(EhFrameLog, "LSDAAddress: ", LSDAAddress);

  auto R = getAddressData(LSDAAddress);
  revng_assert(R, "LSDA not available in any segment");
  llvm::ArrayRef<uint8_t> LSDA = *R;

  DwarfReader<T> LSDAReader(TheArchitecture.type(), LSDA, LSDAAddress);

  uint32_t LandingPadBaseEncoding = LSDAReader.readNextU8();
  MetaAddress LandingPadBase = MetaAddress::invalid();
  if (LandingPadBaseEncoding != dwarf::DW_EH_PE_omit) {
    auto LandingPadBasePointer = LSDAReader.readPointer(LandingPadBaseEncoding);
    LandingPadBase = getGenericPointer<T>(LandingPadBasePointer);
  } else {
    LandingPadBase = FDEStart;
  }

  logAddress(EhFrameLog, "LandingPadBase: ", LandingPadBase);

  uint32_t TypeTableEncoding = LSDAReader.readNextU8();
  if (TypeTableEncoding != dwarf::DW_EH_PE_omit)
    LSDAReader.readULEB128();

  uint32_t CallSiteTableEncoding = LSDAReader.readNextU8();
  uint64_t CallSiteTableLength = LSDAReader.readULEB128();
  uint64_t CallSiteTableEnd = LSDAReader.offset() + CallSiteTableLength;

  while (LSDAReader.offset() < CallSiteTableEnd) {
    // InstructionStart
    LSDAReader.readPointer(CallSiteTableEncoding);

    // InstructionEnd
    LSDAReader.readPointer(CallSiteTableEncoding);

    // LandingPad
    Pointer LandingPadPointer = LSDAReader.readPointer(CallSiteTableEncoding,
                                                       LandingPadBase);
    MetaAddress LandingPad = getCodePointer<T>(LandingPadPointer);

    // Action
    LSDAReader.readULEB128();

    if (LandingPad.isValid()) {
      if (LandingPads.count(LandingPad) == 0)
        logAddress(EhFrameLog, "New landing pad found: ", LandingPad);

      LandingPads.insert(LandingPad);
    }
  }
}

static bool isBetterThan(const Label *NewCandidate, const Label *OldCandidate) {
  if (OldCandidate == nullptr)
    return true;

  if (NewCandidate->address().addressGreaterThan(OldCandidate->address()))
    return true;

  if (NewCandidate->address() == OldCandidate->address()) {
    StringRef OldName = OldCandidate->symbolName();
    if (OldName.size() == 0)
      return true;
  }

  return false;
}

std::string
BinaryFile::nameForAddress(MetaAddress Address, uint64_t Size) const {
  using interval = boost::icl::interval<MetaAddress, compareAddress>;
  std::stringstream Result;
  const auto &SymbolMap = labels();

  auto End = Address.toGeneric() + Size;
  revng_assert(Address.isValid() and End.isValid());
  auto It = SymbolMap.find(interval::right_open(Address, End));
  if (It != SymbolMap.end()) {
    // We have to look for (in order):
    //
    // * Exact match
    // * Contained (non 0-sized)
    // * Contained (0-sized)
    const Label *ExactMatch = nullptr;
    const Label *ContainedNonZeroSized = nullptr;
    const Label *ContainedZeroSized = nullptr;

    for (const Label *L : It->second) {
      // Consider symbols only
      if (not L->isSymbol())
        continue;

      if (L->matches(Address, Size)) {

        // It's an exact match
        ExactMatch = L;
        break;

      } else if (not L->isSizeVirtual() and L->contains(Address, Size)) {

        // It's contained in a not 0-sized symbol
        if (isBetterThan(L, ContainedNonZeroSized))
          ContainedNonZeroSized = L;

      } else if (L->isSizeVirtual() and L->contains(Address, 0)) {

        // It's contained in a 0-sized symbol
        if (isBetterThan(L, ContainedZeroSized))
          ContainedZeroSized = L;
      }
    }

    const Label *Chosen = nullptr;
    if (ExactMatch != nullptr)
      Chosen = ExactMatch;
    else if (ContainedNonZeroSized != nullptr)
      Chosen = ContainedNonZeroSized;
    else if (ContainedZeroSized != nullptr)
      Chosen = ContainedZeroSized;

    if (Chosen != nullptr and Chosen->symbolName().size() != 0) {
      auto Arch = architecture().type();
      Address.dumpRelativeTo(Result,
                             Chosen->address().toPC(Arch),
                             Chosen->symbolName());
      return Result.str();
    }
  }

  // We don't have a symbol to use, just return the address
  Address.dump(Result);
  return Result.str();
}

std::string SegmentInfo::generateName() const {
  // Create name from start and size
  std::stringstream NameStream;
  NameStream << "o_" << (IsReadable ? "r" : "") << (IsWriteable ? "w" : "")
             << (IsExecutable ? "x" : "") << "_0x" << std::hex
             << StartVirtualAddress.address();

  return NameStream.str();
}
