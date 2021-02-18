#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

# Generate classes for the ABI data flow analyses
set(ABIDATAFLOWS_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/ABIDataFlows-header.inc"
  "${CMAKE_CURRENT_SOURCE_DIR}/DeadRegisterArgumentsOfFunction.dot"
  "${CMAKE_CURRENT_SOURCE_DIR}/DeadReturnValuesOfFunctionCall.dot"
  "${CMAKE_CURRENT_SOURCE_DIR}/RegisterArgumentsOfFunctionCall.dot"
  "${CMAKE_CURRENT_SOURCE_DIR}/UsedArgumentsOfFunction.dot"
  "${CMAKE_CURRENT_SOURCE_DIR}/UsedReturnValuesOfFunctionCall.dot"
  "${CMAKE_CURRENT_SOURCE_DIR}/UsedReturnValuesOfFunction.dot")
add_custom_command(OUTPUT ABIDataFlows.h
  COMMAND "${CMAKE_SOURCE_DIR}/scripts/monotone_framework.py"
    --call-arcs ${ABIDATAFLOWS_SOURCES} > ABIDataFlows.h
  DEPENDS "${CMAKE_SOURCE_DIR}/scripts/monotone_framework.py"
           ${ABIDATAFLOWS_SOURCES}
  VERBATIM)
add_custom_target(abidataflows DEPENDS ABIDataFlows.h)

revng_add_analyses_library_internal(revngStackAnalysis
  AAWriterPass.cpp
  ABIDetectionPass.cpp
  ABIIR.cpp
  Cache.cpp
  Element.cpp
  FunctionABI.cpp
  FunctionsSummary.cpp
  IncoherentCallsAnalysis.cpp
  IndirectBranchInfoPrinterPass.cpp
  InterproceduralAnalysis.cpp
  Intraprocedural.cpp
  PromoteGlobalToLocalVars.cpp
  SegregateDirectStackAccesses.cpp
  StackAnalysis.cpp)

target_link_libraries(revngStackAnalysis
  revngBasicAnalyses
  revngABIAnalyses
  revngSupport
  revngModel)

target_include_directories(revngStackAnalysis
  PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}")

add_dependencies(revngStackAnalysis abidataflows)
