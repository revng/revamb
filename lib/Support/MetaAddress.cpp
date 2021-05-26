/// \file MetaAddress.cpp
/// \brief Implementation of MetaAddress.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/Constants.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/Support/MetaAddress.h"

using namespace llvm;

const char *MetaAddressTypeHolderName = "metaaddress_type_holder";

Constant *MetaAddress::toConstant(llvm::Type *Type) const {
  using namespace llvm;

  auto *Struct = cast<StructType>(Type);

  auto GetInt = [Struct](unsigned Index, uint64_t Value) {
    return ConstantInt::get(cast<IntegerType>(Struct->getElementType(Index)),
                            Value);
  };

  return ConstantStruct::get(Struct,
                             GetInt(0, this->Epoch),
                             GetInt(1, this->AddressSpace),
                             GetInt(2, this->Type),
                             GetInt(3, this->Address));
}

GlobalVariable *
MetaAddress::createStructVariableInternal(Module *M,
                                          StringRef Name,
                                          StructType *MetaAddressStruct) {
  using namespace llvm;
  return new GlobalVariable(*M,
                            MetaAddressStruct,
                            false,
                            GlobalValue::InternalLinkage,
                            invalid().toConstant(MetaAddressStruct),
                            Name);
}

StructType *MetaAddress::getStruct(Module *M) {
  using namespace llvm;
  auto *TypeHolder = M->getGlobalVariable(MetaAddressTypeHolderName, true);
  return cast<StructType>(TypeHolder->getType()->getPointerElementType());
}

MetaAddress MetaAddress::fromConstant(Value *V) {
  using namespace llvm;
  using namespace MetaAddressType;

  auto *Struct = cast<ConstantStruct>(V);
  revng_assert(Struct->getNumOperands() == 4);

  auto CI = [](Value *V) { return cast<ConstantInt>(V)->getLimitedValue(); };

  MetaAddress Result;
  Result.Epoch = CI(Struct->getOperand(0));
  Result.AddressSpace = CI(Struct->getOperand(1));
  Result.Type = static_cast<Values>(CI(Struct->getOperand(2)));
  Result.Address = CI(Struct->getOperand(3));
  Result.validate();

  return Result;
}

Instruction *MetaAddress::composeIntegerPC(IRBuilder<> &B,
                                           Value *AddressValue,
                                           Value *EpochValue,
                                           Value *AddressSpaceValue,
                                           Value *TypeValue) {
  llvm::Type *Int128 = B.getInt128Ty();
  auto Load128 = [&B, Int128](Value *V) { return B.CreateZExt(V, Int128); };

  auto *Epoch = B.CreateShl(Load128(EpochValue), 64);
  auto *AddressSpace = B.CreateShl(Load128(AddressSpaceValue), 64 + 32);
  auto *Type = B.CreateShl(Load128(TypeValue), 64 + 32 + 16);
  auto *Composed = B.CreateAdd(Load128(AddressValue),
                               B.CreateAdd(Epoch,
                                           B.CreateAdd(AddressSpace, Type)));

  return cast<Instruction>(Composed);
}

MetaAddress MetaAddress::decomposeIntegerPC(ConstantInt *Value) {
  revng_assert(Value->getType()->getBitWidth() == 128);
  const APInt &APValue = Value->getValue();
  uint64_t Lower = (APValue & UINT64_MAX).getLimitedValue();
  uint64_t Upper = APValue.lshr(64).getLimitedValue();

  MetaAddress Result;
  Result.Address = Lower;
  Result.Epoch = Upper & 0xFFFFFFFF;
  Result.AddressSpace = (Upper >> 32) & 0xFFFF;
  Result.Type = static_cast<MetaAddressType::Values>(Upper >> (32 + 16));
  Result.validate();

  return Result;
}

#define SEP ":"

std::string MetaAddress::toString() const {
  if (isInvalid())
    return SEP "Invalid";

  std::string Result;
  {
    raw_string_ostream Stream(Result);
    Stream << "0x" << Twine::utohexstr(Address) << SEP
           << MetaAddressType::toString(type());
    if (not isDefaultEpoch())
      Stream << SEP << Epoch;
    if (not isDefaultAddressSpace())
      Stream << SEP << AddressSpace;
  }

  return Result;
}

MetaAddress MetaAddress::fromString(StringRef Text) {
  if (Text == SEP "Invalid")
    return MetaAddress::invalid();

  MetaAddress Result;

  SmallVector<StringRef, 4> Parts;
  Text.split(Parts, SEP);

  if (Parts.size() < 2 or Parts.size() > 4)
    return MetaAddress::invalid();

  bool Error;
  uint64_t Value;

  Error = Parts[0].getAsInteger(0, Result.Address);
  if (Error)
    return MetaAddress::invalid();

  Result.Type = MetaAddressType::fromString(Parts[1]);
  if (Result.type() == MetaAddressType::Invalid)
    return MetaAddress::invalid();

  Result.Epoch = 0;
  if (Parts.size() >= 3 and Parts[2].size() > 0) {
    Error = Parts[2].getAsInteger(0, Result.Epoch);
    if (Error)
      return MetaAddress::invalid();
  }

  Result.AddressSpace = 0;
  if (Parts.size() == 4 and Parts[3].size() > 0) {
    Error = Parts[3].getAsInteger(0, Result.AddressSpace);
    if (Error)
      return MetaAddress::invalid();
  }

  Result.validate();

  return Result;
}
