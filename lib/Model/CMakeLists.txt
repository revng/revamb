#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_analyses_library_internal(revngModel
  Binary.cpp
  LoadModelPass.cpp
  SerializeModelPass.cpp
  Type.cpp)

target_link_libraries(revngModel
  revngSupport)
