#include "revng/AutoEnforcer/AutoEnforcerLibraryRegistry.h"
#include "revng/AutoEnforcer/CopyEnforcer.h"
#include "revng/AutoEnforcer/RevngEnforcers.h"

using std::string;
using namespace llvm;
using namespace llvm::cl;
using namespace AutoEnforcer;

class StringContainer : public BackingContainer<StringContainer> {
public:
  using TargertContainer = BackingContainersStatus::TargetContainer;
  ~StringContainer() override = default;

  static char ID;

  std::unique_ptr<BackingContainerBase>
  cloneFiltered(const TargertContainer &Container) const final {
    auto ToReturn = std::make_unique<StringContainer>();
    for (const auto &Target : Container)
      ToReturn->insert(Target);
    return ToReturn;
  }

  void insert(const AutoEnforcerTarget &Target) {
    ContainedStrings.insert(toString(Target));
  }

  bool contains(const AutoEnforcerTarget &Target) const final {
    return ContainedStrings.count(toString(Target)) != 0;
  }

  void mergeBackDerived(StringContainer &&Container) override {
    for (auto &S : Container.ContainedStrings)
      ContainedStrings.insert(S);
  }

  bool remove(const AutoEnforcerTarget &Target) override {
    if (contains(Target))
      return false;

    ContainedStrings.erase(toString(Target));
    return true;
  }

  llvm::Error storeToDisk(llvm::StringRef Path) const override {
    std::error_code EC;
    llvm::raw_fd_ostream OS(Path, EC, llvm::sys::fs::CD_CreateAlways);
    if (EC)
      return llvm::createStringError(EC,
                                     "Could not store to file %s",
                                     Path.str().c_str());

    for (const auto &S : ContainedStrings)
      OS << S << "\n";
    return llvm::Error::success();
  }

  llvm::Error loadFromDisk(llvm::StringRef Path) override {
    if (not llvm::sys::fs::exists(Path)) {
      ContainedStrings.clear();
      return llvm::Error::success();
    }

    std::ifstream OS;
    OS.open(Path, std::ios::in);
    if (not OS.is_open())
      return llvm::createStringError(llvm::inconvertibleErrorCode(),
                                     "Could not load file to file %s",
                                     Path.str().c_str());

    std::string S;
    while (getline(OS, S))
      ContainedStrings.insert(S);
    return llvm::Error::success();
  }

  const std::set<std::string> &getStrings() const { return ContainedStrings; }

private:
  static std::string toString(const AutoEnforcerTarget &Target) {
    std::stringstream S;
    AutoEnforcerTarget::dumpQuantifiers(S, Target.getQuantifiers());
    std::string ToInsert = S.str();
    return ToInsert;
  }

  std::set<std::string> ContainedStrings;
};

char StringContainer::ID;

class ExampleRegistryLibrary : public AutoEnforcerLibraryRegistry {

public:
  void registerContainersAndEnforcers(PipelineLoader &Loader) override {
    Loader.registerDefaultConstructibleContainer<StringContainer>("StringContai"
                                                                  "ner");
    Loader.registerEnforcer("CopyEnforcer",
                            CopyEnforcer<StringContainer>(Root));
  }

  void registerKinds(llvm::StringMap<Kind *> &KindDictionary) override {}

  ~ExampleRegistryLibrary() override = default;
};

static ExampleRegistryLibrary Registry;
