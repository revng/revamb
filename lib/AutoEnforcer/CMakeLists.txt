#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(revngAutoEnforcer
  LLVMEnforcer.cpp
  AutoEnforcerErrors.cpp
  AutoEnforcerTarget.cpp
  Pipeline.cpp
  Step.cpp
  InputOutputContract.cpp
  BackingContainers.cpp
  AutoEnforcer.cpp
  PipelineLoader.cpp)

target_link_libraries(revngModel
  revngSupport)
