#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(revngSupport SHARED
  Assert.cpp
  CommandLine.cpp
  Debug.cpp
  DebugHelper.cpp
  ExampleAnalysis.cpp
  FunctionTags.cpp
  IRHelpers.cpp
  MetaAddress.cpp
  OriginalAssemblyAnnotationWriter.cpp
  PathList.cpp
  ProgramCounterHandler.cpp
  ResourceFinder.cpp
  SelfReferencingDbgAnnotationWriter.cpp
  Statistics.cpp)

llvm_map_components_to_libnames(LLVM_SUPPORT Support)

target_link_libraries(revngSupport ${LLVM_SUPPORT})

target_include_directories(revngSupport
  INTERFACE $<INSTALL_INTERFACE:include/>)
