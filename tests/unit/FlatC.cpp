//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <bit>
#define BOOST_TEST_MODULE FlatC
bool init_unit_test();
#include "boost/test/unit_test.hpp"

#include "revng/Model/Binary.h"
#include "revng/Model/Type.h"

using namespace model;

using llvm::cast;
using llvm::dyn_cast;
using llvm::isa;

static TupleTree<model::Binary>
serializeDeserialize(const TupleTree<model::Binary> &T) {

  std::string Buffer;
  T.serialize(Buffer);
  llvm::outs() << "Serialized\n" << Buffer;

  auto Deserialized = TupleTree<model::Binary>::deserialize(Buffer);

  std::string OtherBuffer;
  Deserialized.serialize(OtherBuffer);
  llvm::outs() << "Deserialized\n" << OtherBuffer;

  return Deserialized;
}

static bool checkSerialization(const TupleTree<model::Binary> &T) {
  revng_check(T->verify());
  auto Deserialized = serializeDeserialize(T);
  revng_check(Deserialized->verify());
  return T->Types == Deserialized->Types;
}

BOOST_AUTO_TEST_CASE(PrimitiveTypes) {
  revng_check(PrimitiveType(PrimitiveTypeKind::Void, 0).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Bool, 1).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Bool, 2).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Bool, 4).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Bool, 8).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Bool, 16).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Unsigned, 1).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Unsigned, 2).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Unsigned, 4).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Unsigned, 8).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Unsigned, 16).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Signed, 1).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Signed, 2).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Signed, 4).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Signed, 8).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Signed, 16).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Float, 2).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Float, 4).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Float, 8).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Float, 10).verify());
  revng_check(PrimitiveType(PrimitiveTypeKind::Float, 16).verify());

  auto Bool = PrimitiveType(PrimitiveTypeKind::Bool, 1);
  auto Unsigned = PrimitiveType(PrimitiveTypeKind::Unsigned, 1);
  auto Signed = PrimitiveType(PrimitiveTypeKind::Signed, 1);
  for (uint8_t ByteSize = 0; ByteSize < 20; ++ByteSize) {

    using namespace std::string_literals;

    Bool.ByteSize = ByteSize;
    Bool.Name = "bool"s + std::to_string(8 * ByteSize) + "_t"s;

    Unsigned.ByteSize = ByteSize;
    Unsigned.Name = "uint"s + std::to_string(8 * ByteSize) + "_t"s;

    Signed.ByteSize = ByteSize;
    Signed.Name = "int"s + std::to_string(8 * ByteSize) + "_t"s;

    if (std::has_single_bit(ByteSize)) {
      revng_check(Bool.verify());
      revng_check(Signed.verify());
      revng_check(Unsigned.verify());
    } else {
      revng_check(not Bool.verify());
      revng_check(not Signed.verify());
      revng_check(not Unsigned.verify());
    }
  }

  auto Float = PrimitiveType(PrimitiveTypeKind::Float, 2);
  for (uint8_t ByteSize = 0; ByteSize < 20; ++ByteSize) {
    using namespace std::string_literals;

    Float.ByteSize = ByteSize;
    Float.Name = "float"s + std::to_string(8 * ByteSize) + "_t"s;

    if (ByteSize == 2 or ByteSize == 4 or ByteSize == 8 or ByteSize == 10
        or ByteSize == 16)
      revng_check(Float.verify());
    else
      revng_check(not Float.verify());
  }
}

using model::PrimitiveTypeKind::Signed;
using model::PrimitiveTypeKind::Void;

BOOST_AUTO_TEST_CASE(EnumTypes) {
  revng_check(not EnumType().verify());
  revng_check(not EnumType("somename").verify());

  TupleTree<model::Binary> T;

  // Insert the type that we want to use as underlying type
  PrimitiveType *Int32 = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Signed, 4));
    revng_check(New);
    revng_check(T->Types.size() == 1);
    Int32 = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(Int32 and Int32->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  EnumType *Enum = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<EnumType>("the_enum"));
    revng_check(New);
    revng_check(T->Types.size() == 2);
    Enum = cast<EnumType>(T->Types.at((*It)->key()).get());
    revng_check(Enum);
  }

  // The enum does not verify if we don't define a valid underlying type and
  // at least one enum entry
  Enum->UnderlyingType = makeTypePath(Int32);
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // With a valid underlying type and at least one entry we're good, but we
  // have to initialize all the cross references in the tree.
  EnumEntry Entry = EnumEntry{ 0, "value0" };
  revng_check(Entry.verify());

  // Entries with no names are not allowed
  revng_check(not EnumEntry(0, "").verify());

  // After inserting at least an entry the enum is good, but until all the
  // cross reference in the tuple tree are not initialized it still does not
  // verify.
  revng_check(Enum->Entries.insert(Entry).second);
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // Do the initialization
  T.initializeReferences();

  // Now we're should be good
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Inserting an alias is ok
  revng_check(Enum->Entries.at(0).Aliases.insert("value_0_alias").second);
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Inserting an alias with the same name of the Name succeeds but is bad
  revng_check(Enum->Entries.at(0).Aliases.insert("value0").second);
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // But if we remove it we're good again.
  revng_check(Enum->Entries.at(0).Aliases.erase("value0"));
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Inserting an empty-name alias succeeds but is bad
  revng_check(Enum->Entries.at(0).Aliases.insert("").second);
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // But if we remove it we're good again.
  revng_check(Enum->Entries.at(0).Aliases.erase(""));
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // We cannot insert other entries with the same value, but we can insert new
  // entries with different values.
  revng_check(Enum->Entries.size() == 1);
  revng_check(not Enum->Entries.insert(EnumEntry{ 0, "other_value0" }).second);
  revng_check(Enum->Entries.size() == 1);
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(Enum->Entries.insert(EnumEntry{ 1, "value_1" }).second);
  revng_check(Enum->Entries.size() == 2);
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Inserting two entries with the same name succceds but it's bad.
  revng_check(Enum->Entries.insert(EnumEntry{ 5, "some_value" }).second);
  revng_check(Enum->Entries.size() == 3);
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));
  revng_check(Enum->Entries.insert(EnumEntry{ 7, "some_value" }).second);
  revng_check(Enum->Entries.size() == 4);
  revng_check(not Enum->verify());
  revng_check(not T->verify());
  // But if we remove the dupicated entry we're good again
  revng_check(Enum->Entries.erase(7));
  revng_check(Enum->Entries.size() == 3);
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // But if we break the underlying, making it point to a type that does not
  // exist, we're not good anymore
  Enum->UnderlyingType = TypePath::fromString("/Types/Typedef-42");
  T.initializeReferences();
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // Also we set the underlying type to a valid type, but that is not a
  // primitive integer type, we are not good
  Enum->UnderlyingType = makeTypePath(Enum);
  T.initializeReferences();
  revng_check(not Enum->verify());
  revng_check(not T->verify());

  // If we put back the proper underlying type it verifies.
  Enum->UnderlyingType = makeTypePath(Int32);
  T.initializeReferences();
  revng_check(Enum->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // But if we clear the entries it does not verify anymore
  Enum->Entries.clear();
  revng_check(not Enum->verify());
  revng_check(not T->verify());
}

BOOST_AUTO_TEST_CASE(TypedefTypes) {
  revng_check(not TypedefType().verify());
  revng_check(not TypedefType("somename").verify());

  TupleTree<model::Binary> T;

  // Insert the type that we want to use as underlying type
  PrimitiveType *Int32 = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Signed, 4));
    revng_check(New);
    revng_check(T->Types.size() == 1);
    Int32 = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(Int32 and Int32->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert the typedef
  TypedefType *Typedef = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<TypedefType>("pid_t"));
    revng_check(New);
    revng_check(T->Types.size() == 2);
    Typedef = cast<TypedefType>(T->Types.at((*It)->key()).get());
    revng_check(Typedef);
  }

  // The pid_t typedef refers to the int32_t
  Typedef->UnderlyingType = { makeTypePath(Int32), {} };

  revng_check(not Typedef->verify());
  revng_check(not T->verify());

  // Do the initialization
  T.initializeReferences();
  revng_check(Typedef->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding qualifiers the typedef still verifies
  Typedef->UnderlyingType.Qualifiers.push_back(Qualifier::createConst());
  revng_check(Typedef->verify());
  revng_check(T->verify());
  Typedef->UnderlyingType.Qualifiers.push_back(Qualifier::createArray(42));
  revng_check(Typedef->verify());
  revng_check(T->verify());
  Typedef->UnderlyingType.Qualifiers.push_back(Qualifier::createPointer());
  revng_check(Typedef->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Removing qualifiers, the typedef still verifies
  Typedef->UnderlyingType.Qualifiers.clear();
  revng_check(Typedef->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // If the underlying type is the type itself something is broken
  Typedef->UnderlyingType.UnqualifiedType = makeTypePath(Typedef);
  T.initializeReferences();
  revng_check(not Typedef->verify());
  revng_check(not T->verify());
}

BOOST_AUTO_TEST_CASE(StructTypes) {
  revng_check(not StructType().verify());
  revng_check(not StructType("somename").verify());

  TupleTree<model::Binary> T;

  // Insert the type that we want to use as underlying type
  PrimitiveType *Int32 = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Signed, 4));
    revng_check(New);
    revng_check(T->Types.size() == 1);
    Int32 = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(Int32 and Int32->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert void type
  PrimitiveType *VoidT = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Void, 0));
    revng_check(New);
    revng_check(T->Types.size() == 2);
    VoidT = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(VoidT and VoidT->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert the struct
  StructType *Struct = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<StructType>("stuff"));
    revng_check(New);
    revng_check(T->Types.size() == 3);
    Struct = cast<StructType>(T->Types.at((*It)->key()).get());
    revng_check(Struct);
  }

  // Let's make it large, so that we can play around with fields.
  Struct->Size = 1024;

  // Insert field in the struct
  StructField Field0 = StructField{ 0, "fld0", { makeTypePath(Int32), {} } };
  revng_check(Struct->Fields.insert(Field0).second);
  revng_check(not Struct->verify());
  revng_check(not T->verify());

  // Do the initialization
  T.initializeReferences();
  // After proper initialization the struct verifies
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding a new field is valid
  StructField Field1 = StructField{ 4, "fld1", { makeTypePath(Int32), {} } };
  revng_check(Struct->Fields.insert(Field1).second);
  T.initializeReferences();
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Inserting fails if the index is already present
  StructField Field1Bis = StructField{ 4,
                                       "fld1bis",
                                       { makeTypePath(Int32), {} } };
  revng_check(not Struct->Fields.insert(Field1Bis).second);
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Assigning succeeds if even if an index is already present
  StructField Field1Ter = StructField{ 4,
                                       "fld1ter",
                                       { makeTypePath(Int32), {} } };
  revng_check(not Struct->Fields.insert_or_assign(Field1Ter).second);
  T.initializeReferences();
  revng_check(Struct->verify());
  revng_check(Struct->Fields.at(4).Name == "fld1ter");
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding a new field whose position is not consecutive to others builds a
  // struct that is valid
  StructField AnotherField = StructField{ 128,
                                          "anotherfld",
                                          { makeTypePath(Int32), {} } };
  revng_check(Struct->Fields.insert(AnotherField).second);
  T.initializeReferences();
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding a new field that overlaps with another is not valid
  StructField Overlap = StructField{ 129,
                                     "anotherfld",
                                     { makeTypePath(Int32), {} } };
  revng_check(Struct->Fields.insert(Overlap).second);
  T.initializeReferences();
  revng_check(not Struct->verify());
  revng_check(not T->verify());

  // Removing the overlapping field fixes the struct
  revng_check(Struct->Fields.erase(129));
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Erasing a field that's not there fails
  revng_check(not Struct->Fields.erase(129));
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Shrinking the size does not break the struct
  Struct->Size = 132;
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  for (int I = 0; I < 132; ++I) {
    // But shrinking too much breaks it again
    Struct->Size = I;
    revng_check(not Struct->verify());
    revng_check(not T->verify());
  }

  // Fixing the size fixes the struct
  Struct->Size = 132;
  revng_check(Struct->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Struct without fields are invalid
  Struct->Fields.clear();
  revng_check(not Struct->verify());
  revng_check(not T->verify());

  // Struct x cannot have a field with type x
  Struct->Fields.clear();
  StructField Same = StructField{ 0,
                                  "samefield",
                                  { makeTypePath(Struct), {} } };
  revng_check(Struct->Fields.insert(Same).second);
  revng_check(not Struct->verify());
  revng_check(not T->verify());

  // Adding a void field is not valid
  Struct->Fields.clear();
  StructField VoidField = StructField{ 0,
                                       "void_fld",
                                       { makeTypePath(VoidT), {} } };
  revng_check(Struct->Fields.insert(VoidField).second);
  T.initializeReferences();
  revng_check(not Struct->verify());
  revng_check(not T->verify());
}

BOOST_AUTO_TEST_CASE(UnionTypes) {
  revng_check(not UnionType().verify());
  revng_check(not UnionType("somename").verify());

  TupleTree<model::Binary> T;

  // Insert the type that we want to use as underlying type
  PrimitiveType *Int32 = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Signed, 4));
    revng_check(New);
    revng_check(T->Types.size() == 1);
    Int32 = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(Int32 and Int32->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert void type
  PrimitiveType *VoidT = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Void, 0));
    revng_check(New);
    revng_check(T->Types.size() == 2);
    VoidT = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(VoidT and VoidT->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert the union
  UnionType *Union = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<UnionType>("stuff"));
    revng_check(New);
    revng_check(T->Types.size() == 3);
    Union = cast<UnionType>(T->Types.at((*It)->key()).get());
    revng_check(Union);
  }

  // Insert field in the struct
  UnionField Field0 = UnionField{ { makeTypePath(Int32), {} }, "fld0" };
  revng_check(Union->Fields.insert(Field0).second);
  revng_check(not Union->verify());
  revng_check(not T->verify());

  // Do the initialization
  T.initializeReferences();
  // After proper initialization the struct verifies
  revng_check(Union->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding a new field is valid
  {
    UnionField Field1 = UnionField{ { makeTypePath(Int32), {} }, "fld1" };
    const auto [It, New] = Union->Fields.insert(std::move(Field1));
    revng_check(New);
    T.initializeReferences();
  }
  revng_check(Union->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  {
    // Assigning another field in a different position with a duplicated name
    // succeeds, but verification fails.
    UnionField Field1 = UnionField{ { makeTypePath(Int32), {} }, "fld1" };
    const auto [It, New] = Union->Fields.insert(std::move(Field1));
    revng_check(New);
    T.initializeReferences();
    revng_check(Union->Fields.at(It->ID).Name == "fld1");
    revng_check(not Union->verify());
    revng_check(not T->verify());

    // But removing goes back to good again
    revng_check(Union->Fields.erase(It->ID));
    revng_check(Union->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Union without fields are invalid
  Union->Fields.clear();
  revng_check(not Union->verify());
  revng_check(not T->verify());

  // Union x cannot have a field with type x
  Union->Fields.clear();
  UnionField Same = UnionField{ { makeTypePath(Union), {} }, "samefield" };
  revng_check(Union->Fields.insert(Same).second);
  T.initializeReferences();
  revng_check(not Union->verify());
  revng_check(not T->verify());

  // Adding a void field is not valid
  Union->Fields.clear();
  UnionField VoidField = UnionField{ { makeTypePath(VoidT), {} }, "void_fld" };
  revng_check(Union->Fields.insert(VoidField).second);
  T.initializeReferences();
  revng_check(not Union->verify());
  revng_check(not T->verify());
}

BOOST_AUTO_TEST_CASE(FunctionPointerTypes) {
  revng_check(not FunctionPointerType().verify());
  revng_check(not FunctionPointerType("somename").verify());

  TupleTree<model::Binary> T;

  // Insert the type that we want to use as underlying type
  PrimitiveType *Int32 = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Signed, 4));
    revng_check(New);
    revng_check(T->Types.size() == 1);
    Int32 = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(Int32 and Int32->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert void type
  PrimitiveType *VoidT = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<PrimitiveType>(Void, 0));
    revng_check(New);
    revng_check(T->Types.size() == 2);
    VoidT = cast<PrimitiveType>(T->Types.at((*It)->key()).get());
    revng_check(VoidT and VoidT->verify());
    revng_check(T->verify());
    revng_check(checkSerialization(T));
  }

  // Insert the function pointer type
  FunctionPointerType *FunPtr = nullptr;
  {
    const auto &[It, New] = T->Types.insert(makeType<FunctionPointerType>("x"));
    revng_check(New);
    revng_check(T->Types.size() == 3);
    FunPtr = cast<FunctionPointerType>(T->Types.at((*It)->key()).get());
    revng_check(FunPtr);
  }

  // Insert argument in the function type
  ArgumentType Arg0 = ArgumentType{ 0, { makeTypePath(Int32), {} } };
  const auto &[InsertedArgIt, New] = FunPtr->ArgumentTypes.insert(Arg0);
  revng_check(InsertedArgIt != FunPtr->ArgumentTypes.end());
  revng_check(New);
  revng_check(not FunPtr->verify());
  revng_check(not T->verify());

  // Do the initialization
  T.initializeReferences();
  // After proper initialization the function type still does not verify,
  // because it does not have a return type
  revng_check(not FunPtr->verify());
  revng_check(not T->verify());

  QualifiedType RetTy{ makeTypePath(Int32), {} };
  FunPtr->ReturnType = RetTy;

  // Now do the initialization again, and it works.
  T.initializeReferences();
  revng_check(FunPtr->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Adding a new field is valid, and we can have a function pointer with an
  // argument of the same type of itself.
  ArgumentType Arg1 = ArgumentType{ 1, { makeTypePath(Int32), {} } };
  revng_check(FunPtr->ArgumentTypes.insert(Arg1).second);
  T.initializeReferences();
  revng_check(FunPtr->verify());
  revng_check(checkSerialization(T));

  // Inserting an ArgumentType in a position that is already taken fails
  ArgumentType Arg1Bis = ArgumentType{ 1, { makeTypePath(Int32), {} } };
  revng_check(not FunPtr->ArgumentTypes.insert(Arg1Bis).second);
  revng_check(FunPtr->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // Assigning an ArgumentType in a position that is already taken succeeds
  revng_check(not FunPtr->ArgumentTypes.insert_or_assign(Arg1Bis).second);
  // After assigning we need to initialize references, and then it will verify.
  T.initializeReferences();
  revng_check(FunPtr->verify());
  auto &ArgT = FunPtr->ArgumentTypes.at(1);
  revng_check(ArgT.Type.UnqualifiedType == makeTypePath(Int32));
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // FunPtr without argument are valid
  FunPtr->ArgumentTypes.clear();
  revng_check(FunPtr->verify());
  revng_check(T->verify());
  revng_check(checkSerialization(T));

  // an ArgumentType can have the same type as the function type
  ArgumentType Self = ArgumentType{ 0, { makeTypePath(FunPtr) } };
  revng_check(FunPtr->ArgumentTypes.insert(Self).second);
  T.initializeReferences();
  revng_check(FunPtr->verify());
  revng_check(T->verify());

  FunPtr->ArgumentTypes.clear();

  // an function can return the same type as the function type
  FunPtr->ReturnType = Self.Type;
  T.initializeReferences();
  revng_check(FunPtr->verify());
  revng_check(T->verify());
}
