/// \file CSVOffsets.cpp

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/Lift/CSVOffsets.h"
#include "revng/Support/Debug.h"

void writeToLog(Logger<true> &L, const CSVOffsets &O, int /*Ignore*/) {
  L << "Kind: " << CSVOffsets::toString(O.OffsetKind);
  L << " Offsets = { ";
  for (const auto &Offset : O)
    L << Offset << ' ';
  L << "}";
}
