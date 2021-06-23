#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

revng_add_library_internal(StringContainerLibrary SHARED ${CMAKE_CURRENT_LIST_DIR}/StringContainerLibrary.cpp)
target_link_libraries(StringContainerLibrary revngAutoEnforcer)

macro(add_auto_enforcer_test TestName PipelineFile Targets InputsList OutputList Flags)
	add_test(NAME ${TestName} COMMAND ${CMAKE_CURRENT_BINARY_DIR}/bin/revng-autoenforcer ${CMAKE_CURRENT_LIST_DIR}/${PipelineFile} ${Targets} -i ${InputsList} -o ${OutputList} -l $<TARGET_FILE:StringContainerLibrary> -p ${CMAKE_CURRENT_BINARY_DIR}/${TestName} -f ${Flags})
endmacro()

macro(add_auto_enforcer_dump_test TestName PipelineFile) 
	add_test(NAME ${TestName} COMMAND revng-autoenforcer ${CMAKE_CURRENT_LIST_DIR}/${PipelineFile} dc:dc:Root -l $<TARGET_FILE:StringContainerLibrary> -d)
endmacro()

macro(add_auto_enforcer_test_with_output TestName PipelineFile Targets InputsList OutputList Flags ToCheckFile ExpectedOutput)

	add_auto_enforcer_test(${TestName} ${PipelineFile} ${Targets} ${InputsList} ${OutputList} ${Flags})
	add_test(NAME check-${TestName} COMMAND diff ${ToCheckFile} ${ExpectedOutput})

 set_tests_properties(check-${TestName} PROPERTIES DEPENDS ${TestName})
endmacro()

macro(add_auto_enforcer_failing_test TestName PipelineFile Targets InputsList OutputList Flags)

	add_auto_enforcer_test(${TestName} ${PipelineFile} ${Targets} ${InputsList} ${OutputList} ${Flags} ${ExpectedOutput})
    set_tests_properties(${TestName} PROPERTIES WILL_FAIL TRUE)
endmacro()


set(CopyEnforcerTestInputs "FirstStep:Strings1:${CMAKE_CURRENT_LIST_DIR}/CopyEnforcerTestInput.txt")
set(CopyEnforcerTestOutputs "End:Strings2:${CMAKE_CURRENT_BINARY_DIR}/CopyEnforcerTestOutput.txt")
add_auto_enforcer_test_with_output(copy-enforcer-run-test CopyEnforcerTestPipeline.yaml Strings2:Root:Root ${CopyEnforcerTestInputs} ${CopyEnforcerTestOutputs} None CopyEnforcerTestOutput.txt ${CMAKE_CURRENT_LIST_DIR}/CopyEnforcerTestExpected.txt)

add_auto_enforcer_test_with_output(flag-autoenforcer-test CopyEnforcerTestPipeline.yaml Strings2:Root:Root ${CopyEnforcerTestInputs} ${CopyEnforcerTestOutputs} None CopyEnforcerTestOutput.txt ${CMAKE_CURRENT_LIST_DIR}/CopyEnforcerTestExpected.txt)


add_auto_enforcer_failing_test(flag-auto-enforcer-fail-test CopyEnforcerFlagTestPipeline.yaml Strings2:Root:Root ${CopyEnforcerTestInputs} ${CopyEnforcerTestOutputs} EnablingFlags)

add_auto_enforcer_failing_test(missing-pass-autoenforcer-fail-test MissingPassPipelineTest.yaml Strings2:Root:Root ${CopyEnforcerTestInputs} ${CopyEnforcerTestOutputs} DontCare)

add_auto_enforcer_dump_test(llvm-pass-test PassPipelineTest.yaml) 
