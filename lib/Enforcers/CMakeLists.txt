#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(revngEnforcers SHARED
  BinaryContainer.cpp
  CompileModule.cpp
  Lift.cpp
  LinkSupport.cpp
  LinkForTranslation.cpp
  RevngEnforcers.cpp)

llvm_map_components_to_libnames(LLVM_LIBRARIES MC)

target_link_libraries(revngEnforcers
  revngSupport
  revngLift
  revngAutoEnforcer
  ${LLVM_LIBRARIES})
