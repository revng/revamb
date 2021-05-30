#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(revngEnforcers SHARED
  BinaryContainer.cpp
  RevngEnforcers.cpp)

target_link_libraries(revngEnforcers
  revngSupport
  revngAutoEnforcer
  ${LLVM_LIBRARIES})
