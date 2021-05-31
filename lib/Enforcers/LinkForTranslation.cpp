// WIP

#include "llvm/Support/Program.h"

#include "revng/Model/LoadModelPass.h"
#include "revng/Enforcers/BinaryContainer.h"
#include "revng/Enforcers/LinkForTranslation.h"
#include "revng/Enforcers/Lift.h"

using namespace llvm;
using namespace llvm::sys;

extern int main(int argc, char *argv[]);

class ProgramRunner {
private:
  std::string MainExecutable;
  SmallVector<StringRef, 64> Paths;

public:
  ProgramRunner() : MainExecutable(fs::getMainExecutable("", reinterpret_cast<void *>(main))) {
    Paths = { { path::parent_path(MainExecutable) } };

    // Appent PATH
    char *Path = getenv("PATH");
    if (Path == nullptr)
      return;
    StringRef(Path).split(Paths, ":");

  }

  template<typename R>
  void run(llvm::StringRef ProgramName, R Args) {
    auto MaybeProgramPath = findProgramByName(ProgramName, Paths);
    revng_assert(MaybeProgramPath);

    // Prepare actual arguments
    std::vector<StringRef> StringRefs { *MaybeProgramPath };
    for (std::string &Arg : Args)
      StringRefs.push_back(Arg);

    // Invoke linker
    int ExitCode = ExecuteAndWait(StringRefs[0], StringRefs);
    revng_assert(ExitCode == 0);
  }

};

static ProgramRunner Runner;

void AutoEnforcer::LinkForTranslationEnforcer::run(DefaultLLVMContainer &M,
                                                   BinaryContainer &InputBinary,
                                                   BinaryContainer &ObjectFile,
                                                   BinaryContainer &OutputBinary) {
  const auto &Model = loadModel(M.getModule());

  const size_t PageSize = 4096;

  BinaryContainer LinkerOutput;
  
  std::vector<std::string> Args = {
    "-lz",
    "-lm",
    "-lrt",
    "-lpthread",
    "-L./",
    "-no-pie",
    ObjectFile.path(),
    "-o", LinkerOutput.path(),
    ("-Wl,-z,max-page-size=" + Twine(PageSize)).str(),
    "-fuse-ld=bfd"
  };

  revng_assert(Model->Segments.size() > 0);
  uint64_t Min = Model->Segments.begin()->Start.address();
  uint64_t Max = Model->Segments.begin()->End.address();;
  for (const auto &Segment : Model->Segments) {
    Min = std::min(Min, Segment.Start.address());
    Max = std::max(Max, Segment.End.address());

    // Force section address
    Args.push_back((Twine("-Wl,--section-start=.") + Segment.Name).str());
  }
  
  // Force text to start on the page after all the original program segments
  auto PageAddress = PageSize * ((Max + PageSize - 1) / PageSize);
  Args.push_back(("-Wl,-Ttext-segment=" + Twine(PageAddress).str()));

  // Force a page before the lowest original address for the ELF header
  Args.push_back(("-Wl,--section-start=.elfheaderhelper=" +
                  Twine::utohexstr(Min - 1)).str());

  // Link required dynamic libraries
  Args.push_back("-Wl,--no-as-needed");
  for (std::string &ImportedLibrary : Model->ImportedLibraries)
    Args.push_back((Twine("-l") + ImportedLibrary).str());
  Args.push_back("-Wl,--as-needed");

  // Prepare actual arguments
  Runner.run("c++", Args);

  // Invoke revng-merge-dynamic
  Args = {
    "merge-dynamic",
    LinkerOutput.path(),
    InputBinary.path(),
    OutputBinary.path()
  };

  // Add base
  if (BaseAddress.getNumOccurrences() > 0) {
    Args.push_back("--base");
    Args.push_back(Twine::utohexstr(BaseAddress).str());
  }

  Runner.run("revng", Args);
}
