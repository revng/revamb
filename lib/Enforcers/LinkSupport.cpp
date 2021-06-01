// WIP

#include "revng/Enforcers/LinkSupport.h"
#include "revng/Support/ResourceFinder.h"

void AutoEnforcer::LinkSupportEnforcer::run(
  DefaultLLVMContainer &TargetContainer) {
  // WIP: option for trace
  const char *SupportConfig = "normal";
  // WIP
  std::string ArchName = "mips";
  std::string SupportSearchPath = ("/share/revng/support-" + ArchName + "-"
                                   + SupportConfig + ".ll");
  auto OptionalSupportPath = revng::ResourceFinder.findFile(SupportSearchPath);
  revng_assert(OptionalSupportPath.has_value(),
               "Cannot find the support module");
  std::string SupportPath = OptionalSupportPath.value();

  // WIP: load
  // WIP: link
}
