/// \file CSVAliasAnalysis.cpp
/// \brief Decorate memory accesses with information about CSV aliasing, and
///        optionally segregate direct stack accesses from all other memory
///        accesses through additional alias information.

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include "revng/BasicAnalyses/CSVAliasAnalysis.h"

using namespace llvm;

class SegregateDirectStackAccesses {
  CSVAliasAnalysisInterface *CSVAA;
  std::vector<Instruction *> DirectStackAccesses;
  std::vector<Instruction *> NotDirectStackAccesses;

public:
  SegregateDirectStackAccesses(CSVAliasAnalysisInterface *CSVAA) :
    CSVAA(CSVAA) {}
  void segregateAccesses(Function &F);

private:
  void decorateStackAccesses(Function &F);
};

template<bool SegregateStackAccesses>
using CSVAAP = CSVAliasAnalysisPass<SegregateStackAccesses>;

template<bool SegregateStackAccesses>
void CSVAAP<SegregateStackAccesses>::initializeAliasInfo(Module &M) {
  LLVMContext &Context = M.getContext();
  QuickMetadata QMD(Context);
  MDBuilder MDB(Context);

  AliasDomain = MDB.createAliasScopeDomain("revngAliasDomain");

  std::set<GlobalVariable *> CSVs;
  NamedMDNode *NamedMD = M.getOrInsertNamedMetadata("revng.csv");
  auto *Tuple = cast<MDTuple>(NamedMD->getOperand(0));
  for (const MDOperand &Operand : Tuple->operands()) {
    auto *CSV = cast<GlobalVariable>(QMD.extract<Constant *>(Operand.get()));
    CSVs.insert(CSV);
  }

  const auto *PCH = GCBI->programCounterHandler();
  for (GlobalVariable *PCCSV : PCH->pcCSVs())
    CSVs.insert(PCCSV);

  // Build alias scopes
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];

    std::string Name = CSV->getName();
    MDNode *CSVScope = MDB.createAliasScope(Name, AliasDomain);
    AliasInfo.AliasScope = CSVScope;
    AllCSVScopes.push_back(CSVScope);
    MDNode *CSVAliasSet = MDNode::get(Context,
                                      ArrayRef<Metadata *>({ CSVScope }));
    AliasInfo.AliasSet = CSVAliasSet;
  }

  // Build noalias sets
  for (const GlobalVariable *CSV : CSVs) {
    CSVAliasInfo &AliasInfo = CSVAliasInfoMap[CSV];
    std::vector<Metadata *> OtherCSVScopes;
    for (const auto &Q : CSVAliasInfoMap)
      if (Q.first != CSV)
        OtherCSVScopes.push_back(Q.second.AliasScope);

    MDNode *CSVNoAliasSet = MDNode::get(Context, OtherCSVScopes);
    AliasInfo.NoAliasSet = CSVNoAliasSet;
  }
}

template<bool SegregateStackAccesses>
void CSVAAP<SegregateStackAccesses>::decorateMemoryAccesses(Instruction &I) {
  Value *Ptr = nullptr;

  if (auto *L = dyn_cast<LoadInst>(&I))
    Ptr = L->getPointerOperand();
  else if (auto *S = dyn_cast<StoreInst>(&I))
    Ptr = S->getPointerOperand();
  else
    return;

  // Check if the pointer is a CSV
  if (auto *GV = dyn_cast<GlobalVariable>(Ptr)) {
    auto It = CSVAliasInfoMap.find(GV);
    if (It != CSVAliasInfoMap.end()) {
      // Set alias.scope and noalias metadata
      I.setMetadata(LLVMContext::MD_alias_scope, It->second.AliasSet);
      I.setMetadata(LLVMContext::MD_noalias, It->second.NoAliasSet);
      return;
    }
  }

  // It's not a CSV memory access, set noalias info
  MDNode *MemoryAliasSet = MDNode::get(I.getContext(), AllCSVScopes);
  I.setMetadata(LLVMContext::MD_noalias,
                MDNode::concatenate(I.getMetadata(LLVMContext::MD_noalias),
                                    MemoryAliasSet));
}

template<bool SegregateStackAccesses>
PreservedAnalyses
CSVAAP<SegregateStackAccesses>::run(Function &F, FunctionAnalysisManager &FAM) {
  // Get the result of the GCBI analysis
  auto &M = *F.getParent();
  auto &MAM = FAM.getResult<ModuleAnalysisManagerFunctionProxy>(F).getManager();
  GCBI = MAM.getCachedResult<GeneratedCodeBasicInfoAnalysis>(M);
  if (!GCBI)
    GCBI = &(FAM.getResult<GeneratedCodeBasicInfoAnalysis>(F));
  revng_assert(GCBI != nullptr);

  // Initialize the CSV alias info.
  initializeAliasInfo(M);

  if constexpr (SegregateStackAccesses) {
    SegregateDirectStackAccesses SDSA(
      static_cast<CSVAliasAnalysisInterface *>(this));
    SDSA.segregateAccesses(F);
  }

  // Decorate the IR with CSV alias info
  for (BasicBlock &BB : F)
    for (Instruction &I : BB)
      decorateMemoryAccesses(I);

  return PreservedAnalyses::none();
}

using SDSA = SegregateDirectStackAccesses;

void SDSA::segregateAccesses(Function &F) {
  Value *SP = nullptr;

  for (BasicBlock &BB : F) {
    for (Instruction &I : BB) {
      Value *V = nullptr;

      // First stage: differentiate accesses and add them
      // onto their respective bucket
      if (auto *LI = dyn_cast<LoadInst>(&I)) {
        V = skipCasts(LI->getPointerOperand());

        // Continue going back if the value is a GEP
        if (isa<GetElementPtrInst>(V))
          V = skipCasts(cast<GetElementPtrInst>(V)->getPointerOperand());

        if (V == CSVAA->GCBI->spReg())
          SP = LI;

        // Put everything that is not a direct access on the stack
        // onto the bucket `NotDirectStackAccesses`. Load/store
        // that access the CSVs will have their alias info added later.
        if (V == SP)
          DirectStackAccesses.push_back(&I);
        else
          NotDirectStackAccesses.push_back(&I);
      } else if (auto *SI = dyn_cast<StoreInst>(&I)) {
        V = skipCasts(SI->getPointerOperand());

        if (V == SP) {
          DirectStackAccesses.push_back(&I);
        } else if (auto *GEP = dyn_cast<GetElementPtrInst>(V)) {
          if (GEP->hasAllConstantIndices())
            DirectStackAccesses.push_back(&I);
        } else {
          NotDirectStackAccesses.push_back(&I);
        }
      }

      // Second stage: transform the I2P + ADD + P2I into a GEP
      if (I.getOpcode() == Instruction::Add) {
        ConstantInt *Offset = nullptr;
        Value *Ptr = nullptr;
        if (skipCasts(Ptr = I.getOperand(0)) == SP)
          Offset = dyn_cast<ConstantInt>(I.getOperand(1));
        else if (skipCasts(Ptr = I.getOperand(1)) == SP)
          Offset = dyn_cast<ConstantInt>(I.getOperand(0));

        if (Ptr && Offset) {
          IRBuilder<> IRB(&I);
          auto *ITP = IRB.CreateIntToPtr(Ptr, IRB.getInt8PtrTy());
          auto *GEP = IRB.CreateGEP(IRB.getInt8Ty(), ITP, Offset);
          auto *PTI = IRB.CreatePtrToInt(GEP, I.getType());
          I.replaceAllUsesWith(PTI);
        }
      }
    }
  }

  decorateStackAccesses(F);
}

void SDSA::decorateStackAccesses(Function &F) {
  auto &Context = F.getContext();
  MDBuilder MDB(Context);

  auto *DirectStackAccessScope = MDB.createAliasScope("DirectStackAccessScope",
                                                      CSVAA->AliasDomain);
  auto *NotDirectStackAccessScope = MDB.createAliasScope("Not("
                                                         "DirectStackAccessScop"
                                                         "e)",
                                                         CSVAA->AliasDomain);

  auto *DSASet = MDNode::get(Context,
                             ArrayRef<Metadata *>({ DirectStackAccessScope }));
  auto *NDSASet = MDNode::get(Context,
                              ArrayRef<Metadata *>(
                                { NotDirectStackAccessScope }));

  for (auto *I : DirectStackAccesses) {
    I->setMetadata(LLVMContext::MD_alias_scope, DSASet);
    I->setMetadata(LLVMContext::MD_noalias, NDSASet);
  }

  for (auto *I : NotDirectStackAccesses) {
    I->setMetadata(LLVMContext::MD_alias_scope, NDSASet);
    I->setMetadata(LLVMContext::MD_noalias, DSASet);
  }
}

template class CSVAliasAnalysisPass<false>;
template class CSVAliasAnalysisPass<true>;
