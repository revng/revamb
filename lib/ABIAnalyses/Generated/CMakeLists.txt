#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

# Code generation requirements


# Generate headers for ABI analyses
set(ABIANALYSIS_TEMPLATE
  "${CMAKE_CURRENT_SOURCE_DIR}/ABIAnalysis.template")

set(ABIANALYSIS_GRAPHS
  "DeadRegisterArgumentsOfFunction"
  "DeadReturnValuesOfFunctionCall"
  "RegisterArgumentsOfFunctionCall"
  "UsedArgumentsOfFunction"
  "UsedReturnValuesOfFunctionCall"
  "UsedReturnValuesOfFunction")

foreach(GRAPH IN LISTS ABIANALYSIS_GRAPHS)
  add_custom_command(OUTPUT "${GRAPH}.h"
    COMMAND "${CMAKE_SOURCE_DIR}/scripts/monotone_framework_lattice.py"
      ${ABIANALYSIS_TEMPLATE} "${CMAKE_CURRENT_SOURCE_DIR}/${GRAPH}.dot" > "${GRAPH}.h"
    DEPENDS "${CMAKE_SOURCE_DIR}/scripts/monotone_framework_lattice.py"
            "${CMAKE_CURRENT_SOURCE_DIR}/${GRAPH}.dot"
            ${ABIANALYSIS_TEMPLATE}
    VERBATIM)
    list(APPEND ABIANALYSIS_HEADERS "${GRAPH}.h")
endforeach()

add_custom_target(abianalysisHeaders DEPENDS ${ABIANALYSIS_HEADERS})
