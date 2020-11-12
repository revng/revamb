#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

# Check if graph_tool is available
execute_process(COMMAND python3 -c "import pygraphviz"
  RESULT_VARIABLE HAS_PYGRAPHVIZ
  OUTPUT_VARIABLE PYGRAPHVIZ_OUTPUT
  ERROR_VARIABLE PYGRAPHVIZ_OUTPUT)

if(NOT HAS_PYGRAPHVIZ EQUAL "0")
  # Don't drop the whitespaces
  message(FATAL_ERROR "
  Cannot find the pygraphviz module: ABIDataFlows.h cannot be generated.
  Please try one of the following commands:

  Debian/Ubuntu:
    apt-get install python3-pygraphviz

  Fedora/Red Hat/CentOS:
    yum install python3-pygraphviz

  pip (current user only):
    pip3 install --user pygraphviz

  In case of successful installation, the following command should succeed with no output:
    python3 -c 'import pygraphviz'")
endif()

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
  COMMAND "${CMAKE_SOURCE_DIR}/scripts/monotone-framework.py"
    --call-arcs ${ABIDATAFLOWS_SOURCES} > ABIDataFlows.h
  DEPENDS "${CMAKE_SOURCE_DIR}/scripts/monotone-framework.py"
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
  revngSupport
  revngModel)

target_include_directories(revngStackAnalysis
  PRIVATE
  "${CMAKE_CURRENT_BINARY_DIR}")

add_dependencies(revngStackAnalysis abidataflows)
