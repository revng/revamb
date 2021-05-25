#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

add_library(StringContainerLibrary SHARED StringContainerLibrary.cpp)
target_link_libraries(StringContainerLibrary revngAutoEnforcer)

macro(add_auto_enforcer_test TestName PipelineFile Targets InputsList OutputList)
	add_test(NAME ${TEST_NAME} COMMAND revng-autoenforcer ${CMAKE_CURRENT_SOURCE_DIR}/${PipelineFile} ${Targets} -i ${InputsList} -o ${OutputList} -l $<TARGET_FILE:StringContainerLibrary> -p ${CMAKE_CURRENT_BINARY_DIR}/${TestName})
endmacro()

set(CopyEnforcerTestInputs "FirstStep:Strings1:${CMAKE_CURRENT_SOURCE_DIR}/CopyEnforcerTestInput.txt")
set(CopyEnforcerTestOutputs "End:Strings2:${CMAKE_CURRENT_BINARY_DIR}/CopyEnforcerTestOutput.txt")
add_auto_enforcer_test(copy-enforcer-run-test CopyEnforcerTestPipeline.yaml Strings2:Root:Root ${CopyEnforcerTestInputs} ${CopyEnforcerTestOutputs})
add_test(NAME copy-enforcer-check-equal-test COMMAND diff ${CMAKE_CURRENT_BINARY_DIR}/CopyEnforcerTestOutput.txt ${CMAKE_CURRENT_SOURCE_DIR}/CopyEnforcerTestExpected.txt)
