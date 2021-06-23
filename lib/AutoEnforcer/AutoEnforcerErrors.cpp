//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/Support/Error.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/AutoEnforcer/AutoEnforcerErrors.h"

using namespace llvm;
using namespace std;
using namespace AutoEnforcer;

char UnsatisfiableRequestError::ID;

void UnsatisfiableRequestError::log(raw_ostream &OS) const {
  OS << "While trying to deduce requirements of request:\n";
  Requested.dump(OS, 1);
  OS << "Could not find a rule to produce:\n";
  LastResort.dump(OS, 1);
}

std::error_code UnsatisfiableRequestError::convertToErrorCode() const {
  return inconvertibleErrorCode();
}

char UnknownAutoEnforcerTarget::ID;

void UnknownAutoEnforcerTarget::log(raw_ostream &OS) const {
  OS << "Could not erase\n";
  for (const auto &Name : Unkown) {
    Name.dump(OS);
    OS << "\n";
  }
  OS << "from: " << BackingContainerName << "\n";
  OS << "Because the queried backing container did not had it";
}

std::error_code UnknownAutoEnforcerTarget::convertToErrorCode() const {
  return inconvertibleErrorCode();
}
