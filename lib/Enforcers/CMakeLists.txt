#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(revngEnforcers SHARED
  BinaryContainer.cpp
  Lift.cpp
  RevngEnforcers.cpp)

target_link_libraries(revngEnforcers
  revngSupport
  revngLift
  revngAutoEnforcer
  ${LLVM_LIBRARIES})
