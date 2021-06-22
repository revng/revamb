#pragma once

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"
#include "llvm/IR/Instruction.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

#include "revng/BasicAnalyses/GeneratedCodeBasicInfo.h"
#include "revng/Model/Binary.h"
#include "revng/Support/Debug.h"
#include "revng/Support/MetaAddress.h"

namespace ABIAnalyses {

using RegisterStateMap = llvm::DenseMap<const llvm::GlobalVariable *,
                                        model::RegisterState::Values>;

struct ABIAnalysesResults {
  // Per function analysis
  RegisterStateMap Arguments;

  // Per call site analysis
  struct CallSiteResults {
    RegisterStateMap Arguments;
    RegisterStateMap ReturnValues;
  };
  std::map<MetaAddress, CallSiteResults> CallSites;

  // Per return analysis
  std::map<MetaAddress, RegisterStateMap> ReturnValues;
  RegisterStateMap FinalReturnValues;

  // Debug methods
  void dump() const debug_function { dump(dbg, ""); }

  template<typename T>
  void dump(T &Output, const char *Prefix) const;
};

ABIAnalysesResults analyzeOutlinedFunction(llvm::Function *,
                                           const GeneratedCodeBasicInfo &,
                                           llvm::Function *,
                                           llvm::Function *);

void finalizeReturnValues(ABIAnalysesResults &);

} // namespace ABIAnalyses
