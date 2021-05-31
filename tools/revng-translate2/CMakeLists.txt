#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_executable(revng-translate2
  Main.cpp)

target_link_libraries(revng-translate2
	revngLift
  revngEnforcers)
