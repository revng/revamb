/// \file AutoEnforcer.cpp
/// \brief Tests for Auto Enforcer

//
// This file is distributed under the MIT License. See LICENSE.md for details.
//

#include <memory>

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/PassSupport.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/YAMLTraits.h"
#include "llvm/Transforms/Utils/Cloning.h"

#include "revng/AutoEnforcer/AutoEnforcer.h"
#include "revng/AutoEnforcer/AutoEnforcerErrors.h"
#include "revng/AutoEnforcer/AutoEnforcerLibraryRegistry.h"
#include "revng/AutoEnforcer/AutoEnforcerTarget.h"
#include "revng/AutoEnforcer/BackingContainerInspector.h"
#include "revng/AutoEnforcer/BackingContainerRegistry.h"
#include "revng/AutoEnforcer/LLVMEnforcer.h"
#include "revng/AutoEnforcer/PipelineLoader.h"

#define BOOST_TEST_MODULE AutoEnforcer
bool init_unit_test();
#include "boost/test/unit_test.hpp"

#include "revng/UnitTestHelpers/UnitTestHelpers.h"

using namespace AutoEnforcer;
using namespace std;
using KE = KindExactness;

static Granularity Root("Root");
static Granularity Function("Function", Root);
static Kind RootKind("RootKind", &Root);
static Kind RootKind2("RootKind2", RootKind, &Root);
static Kind FunctionKind("FunctionKind", &Function);

static std::string CName = "ContainerName";

class MapContainer : public BackingContainer<MapContainer> {
public:
  MapContainer(std::map<AutoEnforcerTarget, int> map) :
    BackingContainer<MapContainer>(), map(std::move(map)) {}
  MapContainer() = default;
  ~MapContainer() override = default;
  using TargertContainer = BackingContainersStatus::TargetContainer;

  unique_ptr<BackingContainerBase>
  cloneFiltered(const TargertContainer &Container) const final {
    return make_unique<MapContainer>(map);
  }

  bool contains(const AutoEnforcerTarget &Target) const final {
    if (Target.getQuantifiers().back().isAll())
      return map.count(AutoEnforcerTarget("f1", FunctionKind))
             and map.count(AutoEnforcerTarget("f2", FunctionKind));
    bool Contained = map.count(Target);
    return Contained;
  }

  void mergeBackDerived(MapContainer &&Container) override {
    for (auto &Pair : Container.map)
      map.insert(std::move(Pair));
  }

  bool remove(const AutoEnforcerTarget &Target) override {
    if (map.find(Target) == map.end())
      return false;

    map.erase(Target);
    return true;
  }

  static char ID;

  auto &get(AutoEnforcerTarget Target) { return map[std::move(Target)]; }
  const auto &get(const AutoEnforcerTarget &Target) const {
    return map.find(std::move(Target))->second;
  }
  auto &getMap() const { return map; }
  auto &getMap() { return map; }

  llvm::Error storeToDisk(llvm::StringRef Path) const override {
    savedData = map;
    return llvm::Error::success();
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) override {
    map = savedData;
    return llvm::Error::success();
  }

private:
  std::map<AutoEnforcerTarget, int> map;
  mutable std::map<AutoEnforcerTarget, int> savedData;
};

char MapContainer::ID;

static const AutoEnforcerTarget Target = { "name", RootKind };

struct Fixture {
  Fixture() {
    RootKind.assign();
    Root.assign();
    FunctionKind.assign();
  }
};

BOOST_AUTO_TEST_SUITE(s, *boost::unit_test::fixture<Fixture>())

BOOST_AUTO_TEST_CASE(BackingContainerIsa) {
  std::map<AutoEnforcerTarget, int> Map;
  Map[Target] = 1;

  auto Ptr = make_unique<MapContainer>(move(Map));
  BackingContainerBase *BasePtr = Ptr.get();
  BOOST_TEST(llvm::isa<MapContainer>(BasePtr));
  BOOST_TEST(llvm::cast<MapContainer>(BasePtr) != nullptr);
  BOOST_TEST(Ptr->get(Target) == 1);
}

BOOST_AUTO_TEST_CASE(BackingContainersCanBeCreated) {
  BackingContainers Containers;
  Containers.add(CName, make_unique<MapContainer>());
  BOOST_TEST(Containers.contains(CName));
  Containers.get<MapContainer>(CName).get(Target) = 1;
  BOOST_TEST(Containers.get<MapContainer>(CName).get(Target) == 1);
}

class TestEnforcer {

public:
  static constexpr auto Name = "TestEnforcer";

  std::vector<AtomicContract> getContract() const {
    return { AtomicContract(RootKind, KE::Exact, 0, RootKind2, 0) };
  }

  void run(const MapContainer &Source, MapContainer &Target) {
    for (const auto &Element : Source.getMap())
      if (&Element.first.getKind() == &RootKind) {
        AutoEnforcerTarget NewTar = { Element.first.getQuantifiers(),
                                      RootKind2 };
        Target.get(NewTar) = Element.second;
      }
  }
};

BOOST_AUTO_TEST_CASE(EnforcerCanBeWrapper) {
  MapContainer Map;
  Map.get({ "name", RootKind }) = 1;
  TestEnforcer Enf;
  Enf.run(Map, Map);
  BOOST_TEST(Map.get({ "name", RootKind2 }) == 1);
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactPassForward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind);

  AtomicContract Contract1(RootKind, KE::Exact);
  Contract1.deduceResults(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactExactForward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind);
  AtomicContract Contract1(RootKind, KE::Exact, 0, RootKind2, 0);
  Contract1.deduceResults(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind2));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractDerivedPassForward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::DerivedFrom);
  Contract1.deduceResults(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind2));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractDerivedExactForward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind, 0);
  Contract1.deduceResults(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactPassBackward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind);

  AtomicContract Contract1(RootKind, KE::Exact);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactExactBackward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::Exact, 0, RootKind2, 0);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractDerivedPassBackward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::DerivedFrom);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind2));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(InputOutputContractDerivedExactBackward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind2, 0);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::DerivedFrom));
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactExactFineGrainedBackward) {
  BackingContainersStatus Targets;
  Targets.add(CName, { "root", "f1" }, FunctionKind);

  AtomicContract Contract1(RootKind, KE::Exact, 0, FunctionKind, 0);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
  BOOST_TEST((Targets[CName][0].getQuantifiers().size() == 1));
  BOOST_TEST((Targets[CName][0].getQuantifiers()[0].getName() == "root"));
}

BOOST_AUTO_TEST_CASE(InputOutputContractExactExactFineGrainedForward) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("root", RootKind);

  AtomicContract Contract1(RootKind, KE::Exact, 0, FunctionKind, 0);
  Contract1.deduceResults(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &FunctionKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
  BOOST_TEST((Targets[CName][0].getQuantifiers().size() == 2));
  BOOST_TEST((Targets[CName][0].getQuantifiers()[0].getName() == "root"));
  BOOST_TEST((Targets[CName][0].getQuantifiers()[1].isAll()));
}

static void checkIfContains(auto &TargetRange, const Kind &K, KE Exact) {
  const auto ToFind = [&K, Exact](const AutoEnforcerTarget &Target) {
    return &Target.getKind() == &K and Target.kindExactness() == Exact;
  };
  BOOST_TEST(find_if(TargetRange, ToFind) != TargetRange.end());
}

BOOST_AUTO_TEST_CASE(InputOutputContractMupltipleInputTest) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);
  Targets[CName].emplace_back("name2", RootKind);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind2, 0);
  Contract1.deduceRequirements(Targets, { CName });

  const auto &ProducedResults = Targets[CName];
  checkIfContains(ProducedResults, RootKind, KE::DerivedFrom);
  checkIfContains(ProducedResults, RootKind, KE::Exact);
}

BOOST_AUTO_TEST_CASE(InputOutputContractPreserved) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);
  Targets[CName].emplace_back("name2", RootKind);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind2, 0, true);
  Contract1.deduceResults(Targets, { CName });
  const auto &ProducedResults = Targets[CName];
  checkIfContains(ProducedResults, RootKind2, KE::Exact);
  checkIfContains(ProducedResults, RootKind, KE::Exact);
  checkIfContains(ProducedResults, RootKind2, KE::Exact);
}

BOOST_AUTO_TEST_CASE(InputOutputContractPreservedBackwardMain) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind2, 0, true);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::DerivedFrom));
}

BOOST_AUTO_TEST_CASE(InputOutputContractPreservedBackwardSecondary) {
  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind);

  AtomicContract Contract1(RootKind, KE::DerivedFrom, 0, RootKind2, 0, true);
  Contract1.deduceRequirements(Targets, { CName });
  BOOST_TEST((&Targets[CName][0].getKind() == &RootKind));
  BOOST_TEST((Targets[CName][0].kindExactness() == KE::Exact));
}

BOOST_AUTO_TEST_CASE(StepCanCloneAndRun) {
  Pipeline Pip;

  BackingContainers Containers;
  Containers.add(CName, make_unique<MapContainer>());
  Containers.get<MapContainer>(CName).get({ "name", RootKind }) = 1;

  Step Step("first_step",
            move(Containers),
            bindEnforcer<TestEnforcer>(CName, CName));

  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);
  auto Result = Step.cloneAndRun({});
  BOOST_TEST(Result.get<MapContainer>(CName).get({ "name", RootKind2 }) == 1);
}

BOOST_AUTO_TEST_CASE(PipelineCanBeManuallyExectued) {
  BackingContainerRegistry Registry;
  Registry.registerDefaultConstructibleFactory<MapContainer>(CName);

  Pipeline Pip;
  Pip.add(Step("first_step",
               Registry.createEmpty(),
               bindEnforcer<TestEnforcer>(CName, CName)));

  Pip.getStartingContainer<MapContainer>(CName).get({ "name", RootKind }) = 1;

  Pip.add(Step("End", Registry.createEmpty()));

  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);

  auto Res = Pip[0].cloneAndRun({});
  BOOST_TEST(Res.get<MapContainer>(CName).get({ "name", RootKind2 }) == 1);
  Pip[0].mergeBackingContainers(std::move(Res));
  const auto &StartingContainer = Pip.getStartingContainer<MapContainer>(CName);
  auto Val = StartingContainer.get({ "name", RootKind2 });
  BOOST_TEST(Val == 1);
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineCanBeRunned) {
  Pipeline Pip;

  BackingContainers Containers;
  Containers.add(CName, make_unique<MapContainer>());
  Containers.get<MapContainer>(CName).get({ "name", RootKind }) = 1;

  Pip.add(Step("first_step",
               move(Containers),
               bindEnforcer<TestEnforcer>(CName, CName)));
  BackingContainers &BCI = Pip.back().getBackingContainers();
  BOOST_TEST(BCI.get<MapContainer>(CName).get({ "name", RootKind }) == 1);

  BackingContainers Containers2;
  Containers2.add(CName, make_unique<MapContainer>());
  Pip.add(Step("End", move(Containers2)));

  BackingContainersStatus Targets;
  Targets[CName].emplace_back("name", RootKind2);
  auto Error = Pip.run(Targets);
  BOOST_TEST(!Error);
  BackingContainers &BC = Pip.back().getBackingContainers();
  BOOST_TEST(BC.get<MapContainer>(CName).get({ "name", RootKind2 }) == 1);
}

class FineGranerEnforcer {

public:
  static constexpr auto Name = "FinedGranedEnforcer";
  std::vector<AtomicContract> getContract() const {
    return { AtomicContract(RootKind, KE::Exact, 0, FunctionKind, 1) };
  }

  void run(const MapContainer &Source, MapContainer &Target) {
    for (const auto &Element : Source.getMap()) {

      if (&Element.first.getKind() != &RootKind)
        continue;

      auto Quantifiers = Element.first.getQuantifiers();
      Quantifiers.emplace_back("f1");
      Target.get({ move(Quantifiers), FunctionKind }) = Element.second;

      Quantifiers = Element.first.getQuantifiers();
      Quantifiers.emplace_back("f2");
      Target.get({ move(Quantifiers), FunctionKind }) = Element.second;
    }
  }
};

BOOST_AUTO_TEST_CASE(SingleElementPipelineBackwardFinedGrained) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;

  BackingContainersStatus Targets;
  Targets.add(CName, { "Root", "f1" }, FunctionKind);

  auto Error = AE.run(Targets);
  BOOST_TEST(!Error);
  const auto &FinalContainer = AE.getFinalContainer<MapContainer>(CName);
  AutoEnforcerTarget FinalTarget({ "Root", "f1" }, FunctionKind);
  auto Val = FinalContainer.get(FinalTarget);

  BOOST_TEST(Val == 1);
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineFailure) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;

  BackingContainersStatus Targets;
  Targets.add(CName, { "RootWRONG", "f1" }, FunctionKind);

  auto Error = AE.run(Targets);
  BOOST_TEST(!!Error);
  consumeError(std::move(Error));
}

static void makeF(llvm::Module &M, llvm::StringRef FName) {

  auto VoidType = llvm::Type::getVoidTy(M.getContext());
  auto *FType = llvm::FunctionType::get(VoidType, {});
  auto F = M.getOrInsertFunction(FName, FType);
  auto *Fun = llvm::dyn_cast<llvm::Function>(F.getCallee());
  auto *BB = llvm::BasicBlock::Create(M.getContext(), "bb", Fun);
  llvm::IRBuilder<> Builder(BB);
  Builder.SetInsertPoint(BB);
  Builder.CreateRet(nullptr);
}

struct FunctionInserterPass : public llvm::ModulePass {
  static char ID;
  FunctionInserterPass() : llvm::ModulePass(ID) {}

  bool runOnModule(llvm::Module &M) override {
    makeF(M, "f1");
    return true;
  }
};
char FunctionInserterPass::ID = '_';

struct IdentityPass : public llvm::ModulePass {
  static char ID;

  IdentityPass() : llvm::ModulePass(ID) {}

  bool runOnModule(llvm::Module &M) override { return true; }
};

char IdentityPass::ID = '_';

static llvm::RegisterPass<IdentityPass> X2("IdentityPass", "IdentityPass");

struct LLVMEnforcerPassFunctionCreator {
  static constexpr auto Name = "Function Creator";

  std::vector<AtomicContract> getContract() const {
    return { AtomicContract(RootKind, KE::Exact, 0, FunctionKind) };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    Manager.add(new FunctionInserterPass());
  }
};

struct LLVMEnforcerPassFunctionIdentity {
  static constexpr auto Name = "Identity";

  std::vector<AtomicContract> getContract() const {
    return { AtomicContract(FunctionKind, KE::Exact) };
  }

  void registerPassess(llvm::legacy::PassManager &Manager) {
    Manager.add(new IdentityPass());
  }
};

BOOST_AUTO_TEST_CASE(SingleElementLLVMPipelineBackwardFinedGrained) {
  llvm::LLVMContext C;

  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerContainerFactory<DefaultLLVMContainerFactory>(CName, C);

  AE.addStep("first_step",
             wrapLLVMPassess(CName,
                             LLVMEnforcerPassFunctionCreator(),
                             LLVMEnforcerPassFunctionIdentity()));
  AE.addStep("End");

  makeF(AE.getStartingContainer<DefaultLLVMContainer>(CName).getModule(),
        "root");

  BackingContainersStatus Targets;
  Targets.add(CName,
              AutoEnforcerTarget({ AutoEnforcerQuantifier("root"),
                                   AutoEnforcerQuantifier("f1") },
                                 FunctionKind));

  auto Error = AE.run(Targets);
  BOOST_TEST(!Error);

  const auto &Final = AE.getFinalContainer<DefaultLLVMContainer>(CName);
  const auto *F = Final.getModule().getFunction("f1");

  BOOST_TEST(F != nullptr);
}

BOOST_AUTO_TEST_CASE(LLVMPureEnforcer) {
  llvm::LLVMContext C;

  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerContainerFactory<DefaultLLVMContainerFactory>(CName, C);

  auto MaybePureLLVMEnforcer = PureLLVMEnforcer::create({ "IdentityPass" });
  BOOST_TEST((!!MaybePureLLVMEnforcer));

  AE.addStep("first_step",
             wrapLLVMPassess(CName, LLVMEnforcerPassFunctionCreator()),
             bindEnforcer(move(*MaybePureLLVMEnforcer), CName));
  AE.addStep("End");

  makeF(AE.getStartingContainer<DefaultLLVMContainer>(CName).getModule(),
        "root");

  BackingContainersStatus Targets;
  Targets.add(CName,
              AutoEnforcerTarget({ AutoEnforcerQuantifier("root"),
                                   AutoEnforcerQuantifier("f1") },
                                 FunctionKind));

  auto Error = AE.run(Targets);
  BOOST_TEST(!Error);

  const auto &Final = AE.getFinalContainer<DefaultLLVMContainer>(CName);
  const auto *F = Final.getModule().getFunction("f1");

  BOOST_TEST(F != nullptr);
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineForwardFinedGrained) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;
  AE.getFinalContainer<MapContainer>(CName).get({ "f1", FunctionKind }) = 1;
  AE.getFinalContainer<MapContainer>(CName).get({ "f2", FunctionKind }) = 1;

  llvm::StringMap<BackingContainersStatus> Invalidations;
  Invalidations["first_step"].add(CName, { "Root" }, RootKind);

  auto Error = AE.deduceInvalidations(Invalidations);
  BOOST_TEST(!Error);
  const auto &
    QuantifOfInvalidated = Invalidations["End"][CName].front().getQuantifiers();
  BOOST_TEST((QuantifOfInvalidated.back().isAll()));
  BOOST_TEST((QuantifOfInvalidated.front().getName() == "Root"));
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineInvalidation) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;
  AE.getFinalContainer<MapContainer>(CName).get({ "f1", FunctionKind }) = 1;
  AE.getFinalContainer<MapContainer>(CName).get({ "f2", FunctionKind }) = 1;

  AutoEnforcerTarget ToKill({ "Root" }, RootKind);
  auto ExpectedInvalidation = AE.getInvalidations(ToKill);
  BOOST_TEST(!!ExpectedInvalidation);
  auto &Invalidations = *ExpectedInvalidation;
  const auto &
    QuantifOfInvalidated = Invalidations["End"][CName].front().getQuantifiers();
  BOOST_TEST((QuantifOfInvalidated.back().isAll()));
  BOOST_TEST((QuantifOfInvalidated.front().getName() == "Root"));
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineWithRemove) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;

  AutoEnforcerTarget ToKill({ "Root" }, RootKind);
  auto Error = AE.invalidate(ToKill);
  BOOST_TEST(!Error);
  auto IsIn = AE.getStartingContainer<MapContainer>(CName).contains(ToKill);
  BOOST_TEST(IsIn == false);
}

BOOST_AUTO_TEST_CASE(PipelineLoaderTest) {
  StepDeclaration SDeclaration{
    "FirstStep", { { "FineGranerEnforcer", { CName, CName } } }
  };
  PipelineDeclaration PDeclaration{ { { CName, "MapContainer" } },
                                    { move(SDeclaration) } };

  PipelineLoader Loader(AutoEnforcerLibraryRegistry::registerAllKinds());
  Loader.registerDefaultConstructibleContainer<MapContainer>("MapContainer");
  Loader.registerEnforcer<FineGranerEnforcer>("FineGranerEnforcer");

  auto MaybeAE = Loader.load(PDeclaration);
  BOOST_TEST(!!MaybeAE);
  auto &AE = *MaybeAE;
  BOOST_TEST((AE[0].getName() == "FirstStep"));
  BOOST_TEST((AE[1].getName() == "End"));

  BackingContainersStatus Targets;
  Targets.add(CName, { "Root", "f1" }, FunctionKind);
  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;

  auto Error = AE.run(Targets);
  BOOST_TEST(!Error);
  const auto &FinalContainer = AE.getFinalContainer<MapContainer>(CName);
  AutoEnforcerTarget FinalTarget({ "Root", "f1" }, FunctionKind);
  auto Val = FinalContainer.get(FinalTarget);

  BOOST_TEST(Val == 1);
}

static const std::string Pipeline(R"(---
                       Containers:
                         - Name:            ContainerName
                           Type:            MapContainer
                       Steps:
                         - Name:            FirstStep
                           Enforcers:
                             - Name:            FineGranerEnforcer
                               UsedContainers:
                                 - ContainerName
                                 - ContainerName
                       )");

BOOST_AUTO_TEST_CASE(PipelineLoaderTestFromYaml) {
  PipelineLoader Loader(AutoEnforcerLibraryRegistry::registerAllKinds());
  Loader.registerDefaultConstructibleContainer<MapContainer>("MapContainer");
  Loader.registerEnforcer<FineGranerEnforcer>("FineGranerEnforcer");
  auto MaybeAutoEnforcer = Loader.load(Pipeline);
  BOOST_TEST(!!MaybeAutoEnforcer);
}

BOOST_AUTO_TEST_CASE(PipelineLoaderTestFromYamlLLVM) {
  llvm::LLVMContext C;
  PipelineLoader Loader(AutoEnforcerLibraryRegistry::registerAllKinds());
  Loader.registerContainerFactory<DefaultLLVMContainerFactory>("LLVMContainer",
                                                               C);
  Loader.registerLLVMEnforcerPass<LLVMEnforcerPassFunctionCreator>("CreateFunct"
                                                                   "ionPa"
                                                                   "ss");
  Loader.registerLLVMEnforcerPass<LLVMEnforcerPassFunctionIdentity>("IdentityPa"
                                                                    "ss");

  std::string LLVMPipeline(R"(---
                       Containers:
                         - Name:            CustomName
                           Type:            LLVMContainer 
                       Steps:
                         - Name:            FirstStep
                           Enforcers:
                             - Name:             LLVMEnforcer
                               UsedContainers:
                                 - CustomName 
                               Passess:
                                 - CreateFunctionPass
                                 - IdentityPass
                       )");

  auto MaybeAutoEnforcer = Loader.load(LLVMPipeline);

  BOOST_TEST(!!MaybeAutoEnforcer);
  if (!MaybeAutoEnforcer)
    llvm::outs() << MaybeAutoEnforcer.takeError();
}

static std::string getCurrentPath() {
  llvm::SmallVector<char, 3> ToReturn;
  llvm::sys::fs::current_path(ToReturn);
  return std::string(ToReturn.begin(), ToReturn.end());
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineStoreToDisk) {
  PipelineRunner AE(AutoEnforcerLibraryRegistry::registerAllKinds());
  AE.registerDefaultConstructibleFactory<MapContainer>(CName);

  AE.addStep("first_step", bindEnforcer<FineGranerEnforcer>(CName, CName));
  AE.addStep("End");

  AE.getStartingContainer<MapContainer>(CName).get({ "Root", RootKind }) = 1;

  BOOST_TEST((!AE.store(getCurrentPath())));

  auto &Container = AE.getStartingContainer<MapContainer>(CName);

  BOOST_TEST((Container.get({ "Root", RootKind }) == 1));
  Container.get({ "Root", RootKind }) = 2;
  BOOST_TEST((Container.get({ "Root", RootKind }) == 2));
  BOOST_TEST((!AE.load(getCurrentPath())));
  BOOST_TEST((Container.get({ "Root", RootKind }) == 1));
}

BOOST_AUTO_TEST_CASE(SingleElementPipelineStoreToDiskWithOverrides) {
  PipelineLoader Loader(AutoEnforcerLibraryRegistry::registerAllKinds());
  Loader.registerDefaultConstructibleContainer<MapContainer>("MapContainer");
  Loader.registerEnforcer<FineGranerEnforcer>("FineGranerEnforcer");
  auto MaybeAutoEnforcer = Loader.load(Pipeline);
  BOOST_TEST(!!MaybeAutoEnforcer);
  auto &AE = *MaybeAutoEnforcer;
  auto MaybeMapping = PipelineFileMapping::parse("FirstStep:ContainerName:"
                                                 "DontCareSourceFile");
  BOOST_TEST(!!MaybeMapping);

  auto &Container = AE.getStartingContainer<MapContainer>(CName);

  Container.get({ "Root", RootKind }) = 1;
  BOOST_TEST((!MaybeMapping->store(AE)));
  Container.get({ "Root", RootKind }) = 2;
  BOOST_TEST((Container.get({ "Root", RootKind }) == 2));
  BOOST_TEST((!MaybeMapping->load(AE)));
  BOOST_TEST((Container.get({ "Root", RootKind }) == 1));
}

class InvalidationEventExample
  : public InvalidationEvent<InvalidationEventExample> {
public:
  static char ID;
};

char InvalidationEventExample::ID;

class InvalidatingKind : public Kind {
public:
  InvalidatingKind() : Kind("Invalidating Kind", &Root) {}
  void deduceInvalidations(const InvalidationEventBase &Event,
                           std::set<GranularityList> &Target) const override {
    if (not llvm::isa<InvalidationEventExample>(Event))
      return;

    Target.insert({ AutoEnforcerQuantifier("f1") });
  }
};

static InvalidatingKind InvalidatingK;

BOOST_AUTO_TEST_CASE(InvalidationListFromEventTest) {
  PipelineLoader Loader(KindsRegisty({ &InvalidatingK }));
  Loader.registerDefaultConstructibleContainer<MapContainer>("MapContainer");
  Loader.registerEnforcer<FineGranerEnforcer>("FineGranerEnforcer");
  auto MaybeAutoEnforcer = Loader.load(Pipeline);
  BOOST_TEST(!!MaybeAutoEnforcer);
  auto &AE = *MaybeAutoEnforcer;
  auto MaybeMapping = PipelineFileMapping::parse("FirstStep:ContainerName:"
                                                 "DontCareSourceFile");
  BOOST_TEST(!!MaybeMapping);

  auto &Container = AE.getStartingContainer<MapContainer>(CName);
  AutoEnforcerTarget T("f1", InvalidatingK, KindExactness::DerivedFrom);
  Container.get(T) = 1;
  InvalidationEventExample Event;
  auto Invalidations = AE.deduceInvalidations(Event);
  BOOST_TEST(Invalidations.size() == 1);
  for (const auto &I : Invalidations) {

    BOOST_TEST(!AE.invalidate(I));
  }

  auto IsIn = AE.getStartingContainer<MapContainer>(CName).contains(T);
  BOOST_TEST(IsIn == false);
}

BOOST_AUTO_TEST_CASE(InvalidationFromEventTest) {
  PipelineLoader Loader(KindsRegisty({ &InvalidatingK }));
  Loader.registerDefaultConstructibleContainer<MapContainer>("MapContainer");
  Loader.registerEnforcer<FineGranerEnforcer>("FineGranerEnforcer");
  auto MaybeAutoEnforcer = Loader.load(Pipeline);
  BOOST_TEST(!!MaybeAutoEnforcer);
  auto &AE = *MaybeAutoEnforcer;
  auto MaybeMapping = PipelineFileMapping::parse("FirstStep:ContainerName:"
                                                 "DontCareSourceFile");
  BOOST_TEST(!!MaybeMapping);

  auto &Container = AE.getStartingContainer<MapContainer>(CName);

  AutoEnforcerTarget T("f1", InvalidatingK, KindExactness::DerivedFrom);
  Container.get(T) = 1;
  InvalidationEventExample Event;
  auto Error = AE.invalidate(Event);

  BOOST_TEST(!Error);

  auto IsIn = AE.getStartingContainer<MapContainer>(CName).contains(T);
  BOOST_TEST(IsIn == false);
}

class InspectableContainerExample
  : public InspectableBackingContainer<InspectableContainerExample> {
public:
  using TargertContainer = BackingContainersStatus::TargetContainer;
  static char ID;

  unique_ptr<BackingContainerBase>
  cloneFiltered(const TargertContainer &Container) const final {
    return make_unique<InspectableContainerExample>(*this);
  }

  void mergeBackDerived(InspectableContainerExample &&Container) override {}

  ~InspectableContainerExample() override = default;

  llvm::Error storeToDisk(llvm::StringRef Path) const override {
    return llvm::Error::success();
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) override {
    return llvm::Error::success();
  }

  std::set<AutoEnforcerTarget> Targets;
};

char InspectableContainerExample::ID;

class ExampleBackingContainerInpsector
  : public BackingContainerInspector<InspectableContainerExample> {
public:
  ExampleBackingContainerInpsector() :
    BackingContainerInspector<InspectableContainerExample>(RootKind) {}
  bool contains(const AutoEnforcerTarget &Target,
                const InspectableContainerExample &Container) const {
    return Container.Targets.count(Target);
  }

  bool remove(const AutoEnforcerTarget &Target,
              InspectableContainerExample &Container) const {

    if (not contains(Target, Container))
      return false;

    Container.Targets.erase(Target);
    return true;
  }
};

static ExampleBackingContainerInpsector Example;

BOOST_AUTO_TEST_CASE(InspectableContainersTest) {
  InspectableContainerExample Example;
  AutoEnforcerTarget T("f1", RootKind, KindExactness::DerivedFrom);
  Example.Targets.insert(T);
  BOOST_TEST(Example.contains(T));
  BOOST_TEST(Example.remove(T));
  BOOST_TEST(not Example.contains(T));
}

BOOST_AUTO_TEST_SUITE_END()
