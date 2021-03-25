#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/Support/Error.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/AutoEnforcer/InputOutputContract.h"

namespace Model {

class UnsatisfiableRequestError
  : public llvm::ErrorInfo<UnsatisfiableRequestError> {
public:
  static char ID;

  UnsatisfiableRequestError(BackingContainersStatus Requested,
                            BackingContainersStatus LastResort) :
    Requested(std::move(Requested)), LastResort(std::move(LastResort)) {}

  std::error_code convertToErrorCode() const override;
  void log(llvm::raw_ostream &OS) const override;

private:
  BackingContainersStatus Requested;
  BackingContainersStatus LastResort;
};

class UnknownAutoEnforcerTarget
  : public llvm::ErrorInfo<UnknownAutoEnforcerTarget> {
public:
  static char ID;

  UnknownAutoEnforcerTarget(llvm::ArrayRef<AutoEnforcerTarget> Unkown,
                            std::string BackingContainerName) :
    Unkown(Unkown.begin(), Unkown.end()),
    BackingContainerName(std::move(BackingContainerName)) {}

  std::error_code convertToErrorCode() const override;
  void log(llvm::raw_ostream &OS) const override;

private:
  std::vector<AutoEnforcerTarget> Unkown;
  std::string BackingContainerName;
};

} // namespace Model
