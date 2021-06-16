//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <bit>
#include <cstddef>
#include <random>
#include <string>
#include <type_traits>

#include "llvm/ADT/SmallSet.h"

#include "revng/Model/Binary.h"
#include "revng/Model/Type.h"

using llvm::cast;
using llvm::dyn_cast_or_null;

namespace model {

static std::set<std::string> CReservedKeywords = {
  // C reserved keywords
  "auto",
  "break",
  "case",
  "char",
  "const",
  "continue",
  "default",
  "do",
  "double",
  "else",
  "enum",
  "extern",
  "float",
  "for",
  "goto",
  "if",
  "inline", // Since C99
  "int",
  "long",
  "register",
  "restrict", // Since C99
  "return",
  "short",
  "signed",
  "sizeof",
  "static",
  "struct",
  "switch",
  "typedef",
  "union",
  "unsigned",
  "volatile",
  "while",
  "_Alignas", // Since C11
  "_Alignof", // Since C11
  "_Atomic", // Since C11
  "_Bool", // Since C99
  "_Complex", // Since C99
  "_Decimal128", // Since C23
  "_Decimal32", // Since C23
  "_Decimal64", // Since C23
  "_Generic", // Since C11
  "_Imaginary", // Since C99
  "_Noreturn", // Since C11
  "_Static_assert", // Since C11
  "_Thread_local", // Since C11
  // Convenience macros
  "alignas",
  "alignof",
  "bool",
  "complex",
  "imaginary",
  "noreturn",
  "static_assert",
  "thread_local",
  // Convenience macros for atomic types
  "atomic_bool",
  "atomic_char",
  "atomic_schar",
  "atomic_uchar",
  "atomic_short",
  "atomic_ushort",
  "atomic_int",
  "atomic_uint",
  "atomic_long",
  "atomic_ulong",
  "atomic_llong",
  "atomic_ullong",
  "atomic_char16_t",
  "atomic_char32_t",
  "atomic_wchar_t",
  "atomic_int_least8_t",
  "atomic_uint_least8_t",
  "atomic_int_least16_t",
  "atomic_uint_least16_t",
  "atomic_int_least32_t",
  "atomic_uint_least32_t",
  "atomic_int_least64_t",
  "atomic_uint_least64_t",
  "atomic_int_fast8_t",
  "atomic_uint_fast8_t",
  "atomic_int_fast16_t",
  "atomic_uint_fast16_t",
  "atomic_int_fast32_t",
  "atomic_uint_fast32_t",
  "atomic_int_fast64_t",
  "atomic_uint_fast64_t",
  "atomic_intptr_t",
  "atomic_uintptr_t",
  "atomic_size_t",
  "atomic_ptrdiff_t",
  "atomic_intmax_t",
  "atomic_uintmax_t",
  // C Extensions
  "_Pragma",
  "asm",
};

static llvm::cl::opt<uint64_t> ModelTypeIDSeed("model-type-id-seed",
                                               llvm::cl::desc("Set the seed "
                                                              "for the "
                                                              "generation of "
                                                              "ID of model "
                                                              "Types"),
                                               llvm::cl::cat(MainCategory),
                                               llvm::cl::init(false));

class RNG {
  std::mt19937_64 generator;
  std::uniform_int_distribution<uint64_t> distribution;

public:
  RNG() :
    generator(ModelTypeIDSeed.getNumOccurrences() ? ModelTypeIDSeed.getValue() :
                                                    std::random_device()()),
    distribution(std::numeric_limits<uint64_t>::min(),
                 std::numeric_limits<uint64_t>::max()) {}

  uint64_t get() { return distribution(generator); }
};

llvm::ManagedStatic<RNG> IDGenerator;

model::Type::Type(TypeKind::Values TK, llvm::StringRef NameRef) :
  model::Type::Type(TK, IDGenerator->get(), NameRef) {
}

bool Qualifier::verify() const {
  switch (Kind) {
  case QualifierKind::Invalid:
    return false;
  case QualifierKind::Pointer:
  case QualifierKind::Const:
    return not Size;
  case QualifierKind::Array:
    return Size;
  }
  return false;
}

static constexpr bool
isValidPrimitiveSize(PrimitiveTypeKind::Values PrimKind, uint8_t BS) {
  switch (PrimKind) {
  case PrimitiveTypeKind::Void:
    return BS == 0;

  case PrimitiveTypeKind::Bool:
  case PrimitiveTypeKind::Unsigned:
  case PrimitiveTypeKind::Signed:
    return BS == 1 or BS == 2 or BS == 4 or BS == 8 or BS == 16;

  case PrimitiveTypeKind::Float:
    return BS == 2 or BS == 4 or BS == 8 or BS == 10 or BS == 16;

  default:
    return false;
  }
  return false;
}

static std::string
makePrimitiveName(PrimitiveTypeKind::Values PrimKind, uint8_t BS) {
  using namespace std::string_literals;
  revng_assert(isValidPrimitiveSize(PrimKind, BS));

  switch (PrimKind) {
  case PrimitiveTypeKind::Void:
    return "void";

  case PrimitiveTypeKind::Bool:
    return "bool"s + std::to_string(BS * 8) + "_t"s;

  case PrimitiveTypeKind::Unsigned:
    return "uint"s + std::to_string(BS * 8) + "_t"s;

  case PrimitiveTypeKind::Signed:
    return "int"s + std::to_string(BS * 8) + "_t"s;

  case PrimitiveTypeKind::Float:
    return "float"s + std::to_string(BS * 8) + "_t"s;

  default:
    revng_abort();
  }
}

PrimitiveType::PrimitiveType(PrimitiveTypeKind::Values PrimKind, uint8_t BS) :
  Type(TypeKind::Primitive, makePrimitiveName(PrimKind, BS)),
  PrimitiveKind(PrimKind),
  ByteSize(BS) {
}

bool EnumEntry::verify() const {
  return not Name.empty() and not Aliases.count(Name) and not Aliases.count("");
}

static bool isOnlyConstQualified(const QualifiedType &QT) {
  if (QT.Qualifiers.empty() or QT.Qualifiers.size() > 1)
    return false;

  return QT.Qualifiers[0].isConstQualifier();
}

struct VoidConstResult {
  bool IsVoid;
  bool IsConst;
};

static VoidConstResult isVoidConst(const QualifiedType *QualType) {
  VoidConstResult Result{ /* IsVoid */ false, /* IsConst */ false };

  bool Done = false;
  while (not Done) {

    // If the argument type is qualified try to get the unqualified version.
    // Warning: we only skip const-qualifiers here, cause the other qualifiers
    // actually produce a different type.
    const Type *UnqualType = nullptr;
    if (not QualType->Qualifiers.empty()) {

      // If it has a non-const qualifier, it can never be void because it's a
      // pointer or array, so we can break out.
      if (not isOnlyConstQualified(*QualType)) {
        Done = true;
        continue;
      }

      // We know that it's const-qualified here, and it only has one
      // qualifier, hence we can skip the const-qualifier.
      Result.IsConst = true;
      if (not QualType->UnqualifiedType.Root)
        return Result;
    }

    UnqualType = QualType->UnqualifiedType.get();

    switch (UnqualType->Kind) {

    // If we still have a typedef in our way, unwrap it and keep looking.
    case TypeKind::Typedef: {
      QualType = &cast<TypedefType>(UnqualType)->UnderlyingType;
    } break;

    // If we have a primitive type, check the name, and we're done.
    case TypeKind::Primitive: {
      auto *P = cast<PrimitiveType>(UnqualType);
      Result.IsVoid = P->PrimitiveKind == PrimitiveTypeKind::Void;
      Done = true;
    } break;

    // In all the other cases it's not void, break from the while.
    default: {
      Done = true;
    } break;
    }
  }
  return Result;
}

model::UnionField::UnionField(const model::QualifiedType &QT,
                              llvm::StringRef NameRef) :
  model::AggregateField::AggregateField(QT, NameRef), ID(IDGenerator->get()) {
}

model::UnionField::UnionField(model::QualifiedType &&QT,
                              llvm::StringRef NameRef) :
  model::AggregateField::AggregateField(std::move(QT), NameRef),
  ID(IDGenerator->get()) {
}

// TODO: this size should depend on the binary we are lifting.
static constexpr const inline uint64_t PointerSize = 8;

static RecursiveCoroutine<uint64_t>
getOrComputeSize(const model::Type *T, TypeSizeCache::TypeSizeMap &SizeCache);

static RecursiveCoroutine<uint64_t>
getOrComputeSize(const model::QualifiedType &QT,
                 TypeSizeCache::TypeSizeMap &SizeCache) {
  // This code assumes that the QualifiedType QT is well formed.
  auto QIt = QT.Qualifiers.begin();
  auto QEnd = QT.Qualifiers.end();

  for (; QIt != QEnd; ++QIt) {

    auto &Q = *QIt;
    switch (Q.Kind) {

    case QualifierKind::Invalid:
      rc_return 0;

    case QualifierKind::Pointer:
      // If we find a pointer, we're done
      rc_return PointerSize;

    case QualifierKind::Array: {
      // The size is equal to (number of elements of the array) * (size of a
      // single element).
      const model::QualifiedType ArrayElem{ QT.UnqualifiedType,
                                            { std::next(QIt), QEnd } };
      rc_return rc_recur model::getOrComputeSize(ArrayElem, SizeCache) * Q.Size;
    }

    case QualifierKind::Const:
      // Do nothing, just skip over it
      ;
    }
  }
  rc_return rc_recur model::getOrComputeSize(QT.UnqualifiedType.get(),
                                             SizeCache);
}

static RecursiveCoroutine<uint64_t>
typeSizeImpl(const model::Type *T, TypeSizeCache::TypeSizeMap &SizeCache) {
  // This code assumes that the type T is well formed.
  uint64_t Size = 0;

  switch (T->Kind) {

  case TypeKind::Primitive: {
    auto *P = cast<PrimitiveType>(T);
    Size = P->ByteSize;
  } break;

  case TypeKind::Enum: {
    auto *U = llvm::cast<EnumType>(T)->UnderlyingType.get();
    Size = rc_recur model::getOrComputeSize(U, SizeCache);
  } break;

  case TypeKind::Typedef: {
    auto *Typedef = llvm::cast<TypedefType>(T);
    Size = rc_recur model::getOrComputeSize(Typedef->UnderlyingType, SizeCache);
  } break;

  case TypeKind::Struct: {
    Size = llvm::cast<StructType>(T)->Size;
  } break;

  case TypeKind::Union: {
    auto *U = llvm::cast<UnionType>(T);
    uint64_t Max = 0ULL;
    for (const auto &Field : U->Fields) {
      auto FieldSize = rc_recur model::getOrComputeSize(Field.FieldType,
                                                        SizeCache);
      Max = std::max(Max, FieldSize);
    }
    Size = Max;
  } break;

  case TypeKind::FunctionPointer: {
    Size = PointerSize;
  } break;
  }
  rc_return Size;
};

static RecursiveCoroutine<uint64_t>
getOrComputeSize(const model::Type *T, TypeSizeCache::TypeSizeMap &SizeCache) {

  auto HintIt = SizeCache.lower_bound(T);
  revng_assert(HintIt == SizeCache.end()
               or not SizeCache.key_comp()(HintIt->first, T));
  if (HintIt != SizeCache.end()
      and not SizeCache.key_comp()(T, HintIt->first)) {
    rc_return HintIt->second;
  }

  uint64_t Size = rc_recur typeSizeImpl(T, SizeCache);
  rc_return SizeCache.emplace_hint(HintIt, T, Size)->second;
}

uint64_t TypeSizeCache::size(const model::Type *T) {
  return getOrComputeSize(T, SizeCache);
}

uint64_t TypeSizeCache::size(const model::QualifiedType &QT) {
  return getOrComputeSize(QT, SizeCache);
}

static RecursiveCoroutine<bool>
getOrComputeVerify(const model::Type *T,
                   VerifyTypeCache::VerifiedSet &Verified,
                   TypeSizeCache &SizeCache);

static RecursiveCoroutine<bool>
getOrComputeVerify(const model::QualifiedType &QT,
                   VerifyTypeCache::VerifiedSet &Verified,
                   TypeSizeCache &SizeCache) {
  auto QIt = QT.Qualifiers.begin();
  auto QEnd = QT.Qualifiers.end();
  bool QualifiersAreVerified = true;
  for (; QIt != QEnd; ++QIt) {
    // Each qualifier needs to verify
    if (not QIt->verify()) {
      QualifiersAreVerified = false;
      break;
    }

    auto NextQIt = std::next(QIt);
    // Check that we have not two consecutive const qualifiers
    if (NextQIt != QEnd and QIt->isConstQualifier()
        and NextQIt->isConstQualifier()) {
      QualifiersAreVerified = false;
      break;
    }
  }

  if (not QualifiersAreVerified or not QT.UnqualifiedType.Root)
    rc_return false;

  model::Type *Underlying = QT.UnqualifiedType.get();
  rc_return Underlying and rc_recur getOrComputeVerify(Underlying,
                                                       Verified,
                                                       SizeCache);
}

static RecursiveCoroutine<bool>
verifyImpl(const PrimitiveType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  bool IsPrimitive = T->Kind == TypeKind::Primitive;
  rc_return IsPrimitive and isValidPrimitiveSize(T->PrimitiveKind, T->ByteSize)
    and (T->Name == makePrimitiveName(T->PrimitiveKind, T->ByteSize));
}

static RecursiveCoroutine<bool>
verifyImpl(const EnumType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  if (T->Kind != TypeKind::Enum or T->Entries.empty())
    rc_return false;

  bool EntriesAreVerified = true;
  llvm::SmallSet<llvm::StringRef, 8> Names;
  for (auto &Entry : T->Entries) {

    if (not Entry.verify()) {
      EntriesAreVerified = false;
      break;
    }

    if (not Names.insert(Entry.Name).second) {
      EntriesAreVerified = false;
      break;
    }
  }

  if (not EntriesAreVerified or not T->UnderlyingType.Root)
    rc_return false;

  auto *Underlying = dyn_cast_or_null<PrimitiveType>(T->UnderlyingType.get());
  rc_return Underlying
    and (Underlying->PrimitiveKind != PrimitiveTypeKind::Unsigned)
    and rc_recur getOrComputeVerify(Underlying, Verified, SizeCache);
}

static RecursiveCoroutine<bool>
verifyImpl(const TypedefType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  if (T->Kind != TypeKind::Typedef
      or not T->UnderlyingType.UnqualifiedType.Root)
    rc_return false;

  model::Type *Underlying = T->UnderlyingType.UnqualifiedType.get();
  rc_return Underlying and (T != Underlying)
    and rc_recur getOrComputeVerify(Underlying, Verified, SizeCache);
}

static RecursiveCoroutine<bool>
getOrComputeVerify(const AggregateField &AF,
                   VerifyTypeCache::VerifiedSet &Verified,
                   TypeSizeCache &SizeCache) {
  rc_return not AF.Name.empty()
    and rc_recur getOrComputeVerify(AF.FieldType, Verified, SizeCache);
}

static RecursiveCoroutine<bool>
verifyImpl(const StructType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  if (T->Kind != TypeKind::Struct or not T->Size or T->Fields.empty())
    rc_return false;

  llvm::SmallSet<llvm::StringRef, 8> Names;
  auto NumFields = T->Fields.size();
  auto FieldIt = T->Fields.begin();
  auto FieldEnd = T->Fields.end();
  bool Result = true;
  for (; FieldIt != FieldEnd; ++FieldIt) {
    auto &Field = *FieldIt;

    if (not Field.FieldType.UnqualifiedType.Root) {
      Result = false;
      break;
    }

    auto *FldType = Field.FieldType.UnqualifiedType.get();
    if (not FldType or T == FldType) {
      Result = false;
      break;
    }

    if (not rc_recur getOrComputeVerify(Field, Verified, SizeCache)) {
      Result = false;
      break;
    }

    if (Field.Offset >= T->Size) {
      Result = false;
      break;
    }

    auto FieldEndOffset = Field.Offset + SizeCache.size(Field.FieldType);
    auto NextFieldIt = std::next(FieldIt);
    if (NextFieldIt != FieldEnd) {
      // If this field is not the last, check that it does not overlap with the
      // following field.
      if (FieldEndOffset > NextFieldIt->Offset) {
        Result = false;
        break;
      }
    } else if (FieldEndOffset > T->Size) {
      // Otherwise, if this field is the last, check that it's not larger than
      // size.
      Result = false;
      break;
    }

    if (isVoidConst(&Field.FieldType).IsVoid) {
      Result = false;
      break;
    }

    bool New = Names.insert(Field.Name).second;
    if (not New) {
      Result = false;
      break;
    }
  }
  rc_return Result;
}

static RecursiveCoroutine<bool>
verifyImpl(const UnionType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  if (T->Kind != TypeKind::Union or T->Fields.empty())
    rc_return false;

  llvm::SmallSet<llvm::StringRef, 8> Names;
  bool FieldsAreVerified = true;
  for (auto &Field : T->Fields) {

    if (not Field.FieldType.UnqualifiedType.Root) {
      FieldsAreVerified = false;
      break;
    }

    auto *FldType = Field.FieldType.UnqualifiedType.get();
    if (not FldType or FldType == T) {
      FieldsAreVerified = false;
      break;
    }

    if (not rc_recur getOrComputeVerify(Field, Verified, SizeCache)) {
      FieldsAreVerified = false;
      break;
    }

    if (not Names.insert(Field.Name).second) {
      FieldsAreVerified = false;
      break;
    }

    if (isVoidConst(&Field.FieldType).IsVoid) {
      FieldsAreVerified = false;
      break;
    }
  }
  rc_return FieldsAreVerified;
}

static RecursiveCoroutine<bool>
verifyImpl(const FunctionPointerType *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {
  if (T->Kind != TypeKind::FunctionPointer
      or not T->ReturnType.UnqualifiedType.Root)
    rc_return false;

  const model::Type *Ret = T->ReturnType.UnqualifiedType.get();
  if (not Ret)
    rc_return false;

  if (T != Ret and not rc_recur getOrComputeVerify(Ret, Verified, SizeCache))
    rc_return false;

  bool ArgsAreVerified = true;
  for (auto &Group : llvm::enumerate(T->ArgumentTypes)) {
    auto &ArgType = Group.value();
    uint64_t ArgPos = Group.index();

    if (ArgType.Pos != ArgPos) {
      ArgsAreVerified = false;
      break;
    }

    if (not ArgType.Type.UnqualifiedType.Root) {
      ArgsAreVerified = false;
      break;
    }

    auto *Arg = ArgType.Type.UnqualifiedType.get();
    if (not Arg) {
      ArgsAreVerified = false;
      break;
    }

    if (Arg == T)
      continue;

    if (not rc_recur getOrComputeVerify(Arg, Verified, SizeCache)) {
      ArgsAreVerified = false;
      break;
    }

    VoidConstResult VoidConst = isVoidConst(&ArgType.Type);
    if (VoidConst.IsVoid) {
      // If we have a void argument it must be the only one, and the function
      // cannot be vararg.
      if (T->ArgumentTypes.size() > 1) {
        ArgsAreVerified = false;
        break;
      }

      // Cannot have const-qualified void as argument.
      if (VoidConst.IsConst) {
        ArgsAreVerified = false;
        break;
      }
    }
  }

  rc_return ArgsAreVerified;
}

bool verifyTypeSystem(const SortedVector<UpcastableType> &Types) {

  VerifyTypeCache VerifyCache;

  // All types on their own should verify
  std::set<std::string> Names;
  for (auto &Type : Types) {
    if (not VerifyCache.verify(Type.get()))
      return false;
    if (not Names.insert(Type->Name).second)
      return false;
  }

  // FIXME: should also check that there are no loops in the type system
  return true;
}

static bool verifyBase(const Type *T) {
  return T and T->ID and not T->Name.empty()
         and not llvm::StringRef(T->Name).contains(' ')
         and not CReservedKeywords.count(T->Name);
}

static RecursiveCoroutine<bool>
verifyImpl(const model::Type *T,
           VerifyTypeCache::VerifiedSet &Verified,
           TypeSizeCache &SizeCache) {

  if (not verifyBase(T))
    rc_return false;

  bool Result = false;

  // We could use upcast() but we'd need to workaround coroutines.
  switch (T->Kind) {
  case TypeKind::Primitive:
    Result = rc_recur verifyImpl(cast<PrimitiveType>(T), Verified, SizeCache);
    break;

  case TypeKind::Enum:
    Result = rc_recur verifyImpl(cast<EnumType>(T), Verified, SizeCache);
    break;

  case TypeKind::Typedef:
    Result = rc_recur verifyImpl(cast<TypedefType>(T), Verified, SizeCache);
    break;

  case TypeKind::Struct:
    Result = rc_recur verifyImpl(cast<StructType>(T), Verified, SizeCache);
    break;

  case TypeKind::Union:
    Result = rc_recur verifyImpl(cast<UnionType>(T), Verified, SizeCache);
    break;

  case TypeKind::FunctionPointer:
    Result = rc_recur verifyImpl(cast<FunctionPointerType>(T),
                                 Verified,
                                 SizeCache);
    break;

  default: // Do nothing;
           ;
  }

  rc_return Result;
}

static RecursiveCoroutine<bool>
getOrComputeVerify(const model::Type *T,
                   VerifyTypeCache::VerifiedSet &Verified,
                   TypeSizeCache &SizeCache) {

  auto HintIt = Verified.lower_bound(T);
  revng_assert(HintIt == Verified.end() or not Verified.key_comp()(*HintIt, T));
  if (HintIt != Verified.end() and not Verified.key_comp()(T, *HintIt))
    rc_return true;

  bool IsVerified = rc_recur verifyImpl(T, Verified, SizeCache);

  if (IsVerified)
    Verified.emplace_hint(HintIt, T);
  rc_return IsVerified;
}

bool VerifyTypeCache::verify(const model::Type *T) {
  TypeSizeCache SizeCache{};
  return getOrComputeVerify(T, Verified, SizeCache);
}

bool VerifyTypeCache::verify(const model::QualifiedType &QT) {
  TypeSizeCache SizeCache{};
  return getOrComputeVerify(QT, Verified, SizeCache);
}

bool Type::verify() const {
  return VerifyTypeCache().verify(this);
}

} // namespace model
