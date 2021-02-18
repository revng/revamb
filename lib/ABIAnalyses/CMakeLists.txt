#
# This file is distributed under the MIT License. See LICENSE.md for details.
#
add_subdirectory(Generated)

revng_add_analyses_library_internal(revngABIAnalyses
  ABIAnalysis.cpp
  DeadRegisterArgumentsOfFunction.cpp
  DeadReturnValuesOfFunctionCall.cpp
  RegisterArgumentsOfFunctionCall.cpp
  UsedArgumentsOfFunction.cpp
  UsedReturnValuesOfFunction.cpp
  UsedReturnValuesOfFunctionCall.cpp)

target_link_libraries(revngABIAnalyses
  revngSupport
  revngBasicAnalyses
  revngModel)

target_include_directories(revngABIAnalyses
  PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}")

add_dependencies(revngABIAnalyses abianalysisHeaders)
