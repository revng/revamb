#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <algorithm>
#include <compare>
#include <optional>
#include <string>

#include "revng/ADT/KeyedObjectTraits.h"
#include "revng/ADT/RecursiveCoroutine.h"
#include "revng/ADT/SortedVector.h"
#include "revng/ADT/UpcastablePointer.h"
#include "revng/ADT/UpcastablePointer/YAMLTraits.h"
#include "revng/Model/TupleTree.h"
#include "revng/Support/Assert.h"
#include "revng/Support/Debug.h"
#include "revng/Support/YAMLTraits.h"

namespace model {

class Binary;

};

namespace model::TypeKind {

/// \brief Enum for identifying different kind of model types.
enum Values {
  Primitive,
  Enum,
  Typedef,
  Struct,
  Union,
  FunctionPointer,
};

} // end namespace model::TypeKind

// Make model::TypeKind yaml-serializable, required for making model::Type
// yaml-serializable as well
template<>
struct llvm::yaml::ScalarEnumerationTraits<model::TypeKind::Values> {
  template<typename IOType>
  static void enumeration(IOType &io, model::TypeKind::Values &Val) {
    io.enumCase(Val, "Primitive", model::TypeKind::Primitive);
    io.enumCase(Val, "Enum", model::TypeKind::Enum);
    io.enumCase(Val, "Typedef", model::TypeKind::Typedef);
    io.enumCase(Val, "Struct", model::TypeKind::Struct);
    io.enumCase(Val, "Union", model::TypeKind::Union);
    io.enumCase(Val, "FunctionPointer", model::TypeKind::FunctionPointer);
  }
};

namespace model::TypeKind {

inline llvm::StringRef getName(model::TypeKind::Values V) {
  return getNameFromYAMLEnumScalar(V);
}

inline model::TypeKind::Values fromName(llvm::StringRef Name) {
  return getValueFromYAMLScalar<model::TypeKind::Values>(Name);
}

} // end namespace model::TypeKind

namespace model {

struct Type;
struct PrimitiveType;
struct EnumType;
struct TypedefType;
struct StructType;
struct UnionType;
struct FunctionPointerType;

} // end namespace model

namespace model::QualifierKind {

/// \brief Enum for identifying different kinds of qualifiers.
//
// Notice that we are choosing to represent pointers and arrays as qualifiers.
// The idea is that a qualifier is something that you can add to a type T to
// obtain another type R, in such a way that if T is fully known also R is fully
// known. In this sense Pointer and Array types are qualified types.
enum Values { Invalid, Pointer, Array, Const };

} // end namespace model::QualifierKind

// Make model::QualifierKind::Values yaml-serializable, required for making
// model::Qualifier yaml-serializable as well
template<>
struct llvm::yaml::ScalarEnumerationTraits<model::QualifierKind::Values> {
  template<typename IOType>
  static void enumeration(IOType &io, model::QualifierKind::Values &Val) {
    io.enumCase(Val, "Invalid", model::QualifierKind::Invalid);
    io.enumCase(Val, "Pointer", model::QualifierKind::Pointer);
    io.enumCase(Val, "Array", model::QualifierKind::Array);
    io.enumCase(Val, "Const", model::QualifierKind::Const);
  }
};

namespace model {

struct Qualifier;
struct QualifiedType;

struct EnumEntry;
struct AggregateField;
struct StructField;
struct UnionField;
struct ArgumentType;

} // end namespace model

// Make model::Type derived types usable with UpcastablePointer
template<>
struct concrete_types_traits<model::Type> {
  using type = std::tuple<model::PrimitiveType,
                          model::EnumType,
                          model::TypedefType,
                          model::StructType,
                          model::UnionType,
                          model::FunctionPointerType>;
};

template<>
struct concrete_types_traits<const model::Type> {
  using type = std::tuple<const model::PrimitiveType,
                          const model::EnumType,
                          const model::TypedefType,
                          const model::StructType,
                          const model::UnionType,
                          const model::FunctionPointerType>;
};

/// \brief Concept to identify all types that are derived from model::Type
template<typename T>
concept FlatCType = std::is_base_of_v<model::Type, T>;

/// \brief Make all types derived from model::Type yaml-serializable like tuples
template<FlatCType T>
struct llvm::yaml::MappingTraits<T> : public TupleLikeMappingTraits<T> {};

/// \brief Base class of model types used for LLVM-style RTTI
struct model::Type {
  using Key = std::pair<model::TypeKind::Values, uint64_t>;

  /// \brief The kind of a model::Type
  TypeKind::Values Kind;

  // TODO: we should switch to a sane UUID implementation
  uint64_t ID;

  /// \brief The name of this type. This is the unique key to identifiy a type.
  std::string Name;

public:
  static bool classof(const Type *T) { return false; }

  Key key() const { return Key{ Kind, ID }; }

protected:
  // Type's constructors are protected because we want to only allow Type's
  // derived classes to instantiate a Type.
  // This effectively makes Type an "abstract" class.
  Type(TypeKind::Values TK, llvm::StringRef NameRef = "");

  Type(TypeKind::Values TK, uint64_t TheID, llvm::StringRef NameRef) :
    Kind(TK), ID(TheID), Name(NameRef) {}

public:
  template<FlatCType T>
  bool operator==(const T &Other) const {
    return key() == Other.key();
  }

  template<FlatCType T>
  bool operator<(const T &Other) const {
    return key() < Other.key();
  }

  bool verify() const debug_function;
};
INTROSPECTION_NS(model, Type, Kind, ID, Name);

namespace model {

using UpcastableType = UpcastablePointer<model::Type>;

template<size_t I = 0>
inline model::UpcastableType
makeTypeWithID(model::TypeKind::Values Kind, uint64_t ID);

} // end namespace model

template<>
struct llvm::yaml::ScalarTraits<model::Type::Key>
  : CompositeScalar<model::Type::Key, '-'> {};

// Make UpcastablePointers to model types usable in KeyedObject containers
template<>
struct KeyedObjectTraits<model::UpcastableType> {
  static model::Type::Key key(const model::UpcastableType &Val) {
    return Val->key();
  }
  static model::UpcastableType fromKey(const model::Type::Key &K) {
    return model::makeTypeWithID(K.first, K.second);
  }
};

/// \brief Make UpcastableType yaml-serializable polymorphically
template<>
struct llvm::yaml::MappingTraits<model::UpcastableType>
  : public PolymorphicMappingTraits<model::UpcastableType> {};

/// \brief A qualifier for a model::Type
struct model::Qualifier {
  QualifierKind::Values Kind = QualifierKind::Invalid;
  uint64_t Size = 0;

  // Kind is not Invalid, Pointer and Const have no Size, Array has Size.
  bool verify() const debug_function;

  static Qualifier createConst() { return { QualifierKind::Const, 0 }; }

  static Qualifier createPointer() { return { QualifierKind::Pointer, 0 }; }

  static Qualifier createArray(uint64_t S) {
    return { QualifierKind::Array, S };
  }

  bool isConstQualifier() const {
    revng_assert(verify());
    return Kind == QualifierKind::Const;
  }

  bool isArrayQualifier() const {
    revng_assert(verify());
    return Kind == QualifierKind::Const;
  }

  bool isPtrQualifier() const {
    revng_assert(verify());
    return Kind == QualifierKind::Const;
  }

  bool operator==(const Qualifier &) const = default;
};
INTROSPECTION_NS(model, Qualifier, Kind, Size);

/// \brief Make Qualifier yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::Qualifier>
  : public TupleLikeMappingTraits<model::Qualifier> {};

// Make std::vector<model::Qualifier> yaml-serializable as a sequence
LLVM_YAML_IS_SEQUENCE_VECTOR(model::Qualifier)

namespace model {

using TypePath = TupleTreeReference<model::Type, model::Binary>;

inline model::TypePath makeTypePath(const model::Type *T) {
  if (T) {
    return TypePath::fromString("/Types/" + getNameFromYAMLScalar(T->key()));
  } else {
    return TypePath::fromString("/Types/");
  }
}

} // end namespace model

/// \brief A qualified version of a model::Type. Can have many nested qualifiers
struct model::QualifiedType {
  TypePath UnqualifiedType = model::makeTypePath(nullptr);
  std::vector<Qualifier> Qualifiers = {};
  bool operator==(const model::QualifiedType &Other) const = default;
};
INTROSPECTION_NS(model, QualifiedType, UnqualifiedType, Qualifiers);

/// \brief Make QualifiedType yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::QualifiedType>
  : public TupleLikeMappingTraits<model::QualifiedType> {};

namespace model::PrimitiveTypeKind {

enum Values {
  Void,
  Bool,
  Unsigned,
  Signed,
  Float,
};

} // end namespace model::PrimitiveTypeKind

// Make model::PrimitiveTypeKind::Values yaml-serializable
template<>
struct llvm::yaml::ScalarEnumerationTraits<model::PrimitiveTypeKind::Values> {
  template<typename IOType>
  static void enumeration(IOType &io, model::PrimitiveTypeKind::Values &Val) {
    io.enumCase(Val, "Void", model::PrimitiveTypeKind::Void);
    io.enumCase(Val, "Bool", model::PrimitiveTypeKind::Bool);
    io.enumCase(Val, "Unsigned", model::PrimitiveTypeKind::Unsigned);
    io.enumCase(Val, "Signed", model::PrimitiveTypeKind::Signed);
    io.enumCase(Val, "Float", model::PrimitiveTypeKind::Float);
  }
};

namespace model::PrimitiveTypeKind {

inline llvm::StringRef getName(model::PrimitiveTypeKind::Values V) {
  return getNameFromYAMLEnumScalar(V);
}

inline model::PrimitiveTypeKind::Values fromName(llvm::StringRef Name) {
  return getValueFromYAMLScalar<model::PrimitiveTypeKind::Values>(Name);
}

} // end namespace model::PrimitiveTypeKind

/// \brief A primitive type in model: sized integers, booleans, floats and void.
struct model::PrimitiveType : public model::Type {
  static constexpr const char *Tag = "!Primitive";
  static constexpr const TypeKind::Values AssociatedKind = TypeKind::Primitive;

  PrimitiveTypeKind::Values PrimitiveKind = model::PrimitiveTypeKind::Void;
  uint8_t ByteSize = 0;

  PrimitiveType(model::PrimitiveTypeKind::Values, uint8_t ByteSize);
  PrimitiveType() : PrimitiveType(model::PrimitiveTypeKind::Void, 0) {}

  static bool classof(const Type *T) { return T->Kind == TypeKind::Primitive; }

  // Not to be used directly, only KeyedObjectTraits should use this
  PrimitiveType(uint64_t ID) : Type(TypeKind::Primitive, ID, "") {}
};
INTROSPECTION_NS(model, PrimitiveType, Kind, ID, Name, PrimitiveKind, ByteSize);

/// \brief KeyedObjectTraits for std::string based on its value
template<>
struct KeyedObjectTraits<std::string>
  : public IdentityKeyedObjectTraits<std::string> {};

/// \brief An entry in a model enum, with a name and a value.
struct model::EnumEntry {

  uint64_t Value;
  std::string Name;
  SortedVector<std::string> Aliases;

public:
  EnumEntry(uint64_t V, llvm::StringRef NameRef) :
    Value(V), Name(NameRef), Aliases() {}

  EnumEntry(uint64_t V) : EnumEntry(V, "") {}

  EnumEntry() : EnumEntry(0ULL) {}

  bool operator==(const EnumEntry &) const = default;

public:
  // The entry should have a non-empty name, the name should not be a valid
  // alias, and there should not be empty aliases.
  bool verify() const debug_function;
};
INTROSPECTION_NS(model, EnumEntry, Value, Name, Aliases);

/// \brief KeyedObjectTraits for model::EnumEntry based on its value
template<>
struct KeyedObjectTraits<model::EnumEntry> {

  static uint64_t key(const model::EnumEntry &E) { return E.Value; }

  static model::EnumEntry fromKey(uint64_t V) { return model::EnumEntry{ V }; }
};

/// \brief Make EnumEntry yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::EnumEntry>
  : public TupleLikeMappingTraits<model::EnumEntry> {};

/// \brief An enum type in model. Enums are actually typedefs of unnamed
/// enums.
struct model::EnumType : public model::Type {
  static constexpr const char *Tag = "!Enum";
  static constexpr const TypeKind::Values AssociatedKind = TypeKind::Enum;

  TypePath UnderlyingType;
  SortedVector<EnumEntry> Entries;

public:
  EnumType(llvm::StringRef NameRef) :
    Type(TypeKind::Enum, NameRef), UnderlyingType(), Entries() {}

  EnumType() : EnumType("") {}

  static bool classof(const Type *T) { return T->Kind == TypeKind::Enum; }

  // Not to be used directly, only KeyedObjectTraits should use this
  EnumType(uint64_t ID) : Type(TypeKind::Enum, ID, "") {}
};
INTROSPECTION_NS(model, EnumType, Kind, ID, Name, UnderlyingType, Entries);

/// \brief A typedef type in model.
struct model::TypedefType : public model::Type {
  static constexpr const char *Tag = "!Typedef";
  static constexpr const TypeKind::Values AssociatedKind = TypeKind::Typedef;

  QualifiedType UnderlyingType;

  TypedefType(llvm::StringRef NameRef, const QualifiedType &U) :
    Type(TypeKind::Typedef, NameRef), UnderlyingType(U) {}

  TypedefType(llvm::StringRef NameRef, QualifiedType &&U) :
    Type(TypeKind::Typedef, NameRef), UnderlyingType(std::move(U)) {}

  TypedefType(llvm::StringRef NameRef) :
    TypedefType(NameRef, QualifiedType()) {}

  TypedefType() : TypedefType("") {}

  static bool classof(const Type *T) { return T->Kind == TypeKind::Typedef; }

  // Not to be used directly, only KeyedObjectTraits should use this
  TypedefType(uint64_t ID) : Type(TypeKind::Typedef, ID, "") {}
};
INTROSPECTION_NS(model, TypedefType, Kind, ID, Name, UnderlyingType);

/// \brief A field of an aggregate type in model, with qualified type and name
struct model::AggregateField {
  QualifiedType FieldType;
  std::string Name;

  AggregateField(const QualifiedType &QT, llvm::StringRef NameRef) :
    FieldType(QT), Name(NameRef) {}

  AggregateField(QualifiedType &&QT, llvm::StringRef NameRef) :
    FieldType(std::move(QT)), Name(NameRef) {}

  AggregateField(const QualifiedType &QT) : AggregateField(QT, "") {}

  AggregateField(QualifiedType &&QT) : AggregateField(std::move(QT), "") {}

  AggregateField() :
    AggregateField(QualifiedType{ TypePath::fromString("/"), {} }) {}

  bool operator==(const AggregateField &) const = default;
};

/// \brief A field of a struct type in model, with offset, qualified type, and
/// name
struct model::StructField : model::AggregateField {
  uint64_t Offset;

  StructField(uint64_t O, llvm::StringRef NameRef, const QualifiedType &QT) :
    AggregateField(QT, NameRef), Offset(O) {}

  StructField(uint64_t O) : StructField(O, "", {}) {}

  StructField() : StructField(0) {}

  bool operator==(const StructField &) const = default;
};
INTROSPECTION_NS(model, StructField, Offset, Name, FieldType);

/// \brief KeyedObjectTraits for model::StructType based on its byte-offset in
/// struct
template<>
struct KeyedObjectTraits<model::StructField> {

  static uint64_t key(const model::StructField &Val) { return Val.Offset; }

  static model::StructField fromKey(const uint64_t &Offset) {
    return model::StructField(Offset);
  }
};

/// \brief Make StructField yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::StructField>
  : public TupleLikeMappingTraits<model::StructField> {};

/// \brief A struct type in model. Structs are actually typedefs of unnamed
/// structs in C.
struct model::StructType : public model::Type {
  static constexpr const char *Tag = "!Struct";
  static constexpr const TypeKind::Values AssociatedKind = TypeKind::Struct;

  SortedVector<StructField> Fields;
  uint64_t Size;

  StructType(llvm::StringRef NameRef) :
    Type(TypeKind::Struct, NameRef), Fields() {}

  StructType() : StructType("") {}

  static bool classof(const Type *T) { return T->Kind == TypeKind::Struct; }

  // Not to be used directly, only KeyedObjectTraits should use this
  StructType(uint64_t ID) : Type(TypeKind::Struct, ID, "") {}
};
INTROSPECTION_NS(model, StructType, Kind, ID, Name, Fields, Size);

/// \brief A field of a union type in model, with position, qualified type, and
/// name
struct model::UnionField : model::AggregateField {
  uint64_t ID;

public:
  UnionField(const QualifiedType &QT, llvm::StringRef NameRef);

  UnionField(QualifiedType &&QT, llvm::StringRef NameRef);

  UnionField() : UnionField({}, "") {}

private:
  UnionField(uint64_t TheID, QualifiedType &&QT, llvm::StringRef NameRef) :
    AggregateField(std::move(QT), NameRef), ID(TheID) {}

public:
  UnionField(uint64_t TheID) : UnionField(TheID, {}, "") {}

  bool operator==(const UnionField &Other) const { return ID == Other.ID; }
};
INTROSPECTION_NS(model, UnionField, ID, Name, FieldType);

/// \brief KeyedObjectTraits for model::UnionField based on its position in the
/// union
template<>
struct KeyedObjectTraits<model::UnionField> {

  static uint64_t key(const model::UnionField &Val) { return Val.ID; }

  static model::UnionField fromKey(const uint64_t &ID) {
    return model::UnionField(ID);
  }
};

/// \brief Make UnionField yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::UnionField>
  : public TupleLikeMappingTraits<model::UnionField> {};

/// \brief A union type in model. Uniont are actually typedefs of unnamed
/// unions in C.
struct model::UnionType : public model::Type {
  static constexpr const char *Tag = "!Union";
  static constexpr const TypeKind::Values AssociatedKind = TypeKind::Union;

  SortedVector<UnionField> Fields;

  UnionType(llvm::StringRef NameRef) :
    Type(TypeKind::Union, NameRef), Fields() {}

  UnionType() : UnionType("") {}

  static bool classof(const Type *T) { return T->Kind == TypeKind::Union; }

  // Not to be used directly, only KeyedObjectTraits should use this
  UnionType(uint64_t ID) : Type(TypeKind::Union, ID, "") {}
};
INTROSPECTION_NS(model, UnionType, Kind, ID, Name, Fields);

/// \brief An argument type in a function pointer type in model.
struct model::ArgumentType {
  uint64_t Pos;
  QualifiedType Type;

  ArgumentType(uint64_t P, const QualifiedType &QT) : Pos(P), Type(QT) {}
  ArgumentType(uint64_t P, QualifiedType &&QT) : Pos(P), Type(std::move(QT)) {}

  ArgumentType(uint64_t P) : ArgumentType(P, {}) {}

  ArgumentType() : ArgumentType(0) {}

  bool operator==(const ArgumentType &) const = default;
};
INTROSPECTION_NS(model, ArgumentType, Pos, Type);

/// \brief KeyedObjectTraits for model::ArgumentType based on its position
template<>
struct KeyedObjectTraits<model::ArgumentType> {

  static uint64_t key(const model::ArgumentType &Val) { return Val.Pos; }

  static model::ArgumentType fromKey(const uint64_t &Pos) {
    return model::ArgumentType(Pos);
  }
};

/// \brief Make ArgumentType yaml-serializable
template<>
struct llvm::yaml::MappingTraits<model::ArgumentType>
  : public TupleLikeMappingTraits<model::ArgumentType> {};

/// \brief A function pointer type in model. Function pointers are actually
/// typedefs.
struct model::FunctionPointerType : public model::Type {
  static constexpr const char *Tag = "!FunctionPointer";
  static constexpr const auto AssociatedKind = TypeKind::FunctionPointer;

  QualifiedType ReturnType;
  SortedVector<ArgumentType> ArgumentTypes;
  // TODO: do we need to support variadic functions? If so, how?
  // FIXME: eventually we should do something about ABI;

  FunctionPointerType(llvm::StringRef NameRef) :
    Type(TypeKind::FunctionPointer, NameRef), ReturnType(), ArgumentTypes() {}

  FunctionPointerType() : FunctionPointerType("") {}

  static bool classof(const Type *T) {
    return T->Kind == TypeKind::FunctionPointer;
  }

  // Not to be used directly, only KeyedObjectTraits should use this
  FunctionPointerType(uint64_t ID) : Type(TypeKind::FunctionPointer, ID, "") {}
};
INTROSPECTION_NS(model,
                 FunctionPointerType,
                 Kind,
                 ID,
                 Name,
                 ReturnType,
                 ArgumentTypes);

namespace model {

template<size_t I>
inline model::UpcastableType
makeTypeWithID(model::TypeKind::Values K, uint64_t ID) {
  using concrete_types = concrete_types_traits_t<model::Type>;
  if constexpr (I < std::tuple_size_v<concrete_types>) {
    using type = std::tuple_element_t<I, concrete_types>;
    if (type::AssociatedKind == K)
      return model::UpcastableType(new type(ID));
    else
      return model::makeTypeWithID<I + 1>(K, ID);
  } else {
    return model::UpcastableType(nullptr);
  }
}

using TypesSet = SortedVector<UpcastableType>;

} // end namespace model

static_assert(validateTupleTree<model::TypesSet>(IsYamlizable),
              "All elements of the model type system must be YAMLizable");

namespace model {

template<FlatCType T, typename... Args>
inline UpcastableType makeType(Args &&...A) {
  return UpcastableType(new T(std::forward<Args>(A)...));
}

class TypeSizeCache {
public:
  using TypeSizeMap = std::map<const model::Type *, uint64_t>;

public:
  void clear() { SizeCache.clear(); }

  uint64_t size(const model::Type *T);

  uint64_t size(const model::UpcastableType &T) { return size(T.get()); }

  uint64_t size(const model::QualifiedType &T);

private:
  TypeSizeMap SizeCache = {};
}; // end class TypeSizeVisitor

class VerifyTypeCache {
public:
  using VerifiedSet = std::set<const model::Type *>;

  void clear() { Verified.clear(); }

  bool verify(const model::Type *T);

  bool verify(const model::UpcastableType &T) { return verify(T.get()); }

  bool verify(const model::QualifiedType &T);

private:
  VerifiedSet Verified = {};
}; // end class VerifyTypeCache

extern bool verifyTypeSystem(const model::TypesSet &Types) debug_function;

} // end namespace model
