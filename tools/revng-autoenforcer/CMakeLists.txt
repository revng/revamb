#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_executable(revng-autoenforcer
  Main.cpp)

target_link_libraries(revng-autoenforcer
  revngEnforcers
	revngAutoEnforcer)
