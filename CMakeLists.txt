#
# This file is distributed under the MIT License. See LICENSE.md for details.
#

cmake_minimum_required(VERSION 3.15.0)

project(revng)

include(cmake/Common.cmake)
install(FILES cmake/revngConfig.cmake cmake/Common.cmake
  DESTINATION share/revng/cmake)

# This has to be first to get highest priority
include_directories(include/)

# Doxygen
find_package(Doxygen)
if(DOXYGEN_FOUND)
  execute_process(COMMAND git ls-files
    WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
    RESULT_VARIABLE GIT_LS_EXIT_CODE
    OUTPUT_VARIABLE GIT_LS_OUTPUT
    ERROR_VARIABLE GIT_LS_OUTPUT_STDERR)

  if(GIT_LS_EXIT_CODE EQUAL "0")
    string(REGEX REPLACE "\n" ";" GIT_LS_OUTPUT "${GIT_LS_OUTPUT}")
    set(DOXYGEN_INPUTS "")
    foreach(FILE ${GIT_LS_OUTPUT})
      set(DOXYGEN_INPUTS "${DOXYGEN_INPUTS} ${CMAKE_SOURCE_DIR}/${FILE}")
    endforeach(FILE)
    configure_file(${CMAKE_SOURCE_DIR}/Doxyfile.in
      ${CMAKE_BINARY_DIR}/Doxyfile @ONLY)
    add_custom_target(doc
      ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/Doxyfile
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Generating API documentation with Doxygen" VERBATIM)
  else()
    message(WARNING "Source directory is not a git repository, disabling Doxygen. Error was: ${GIT_LS_OUTPUT_STDERR}")
  endif()

endif(DOXYGEN_FOUND)

# LLVM CMake stuff
find_package(LLVM REQUIRED CONFIG)
include_directories(${LLVM_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})
llvm_map_components_to_libnames(LLVM_LIBRARIES core support irreader ScalarOpts
	linker Analysis object transformutils InstCombine CodeGen Passes BitWriter InstCombine)

# Build the support module for each architecture and in several configurations
set(CLANG "${LLVM_TOOLS_BINARY_DIR}/clang")

set(SUPPORT_MODULES_CONFIGS "normal;trace")
set(SUPPORT_MODULES_CONFIG_normal "")
set(SUPPORT_MODULES_CONFIG_trace "-DTRACE")

make_directory("${CMAKE_BINARY_DIR}/share/revng/")

foreach(ARCH aarch64 arm mips mipsel x86_64 i386 s390x)
  set(OUTPUT "early-linked-${ARCH}.ll")
  add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
    DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/early-linked.c"
    COMMAND "${CLANG}"
    ARGS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/early-linked.c"
         -o "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
         -S -emit-llvm -g
         -DTARGET_${ARCH}
         -I"${CMAKE_CURRENT_SOURCE_DIR}/runtime"
         -I"${CMAKE_CURRENT_SOURCE_DIR}/include")
  add_custom_target("early-linked-module-${OUTPUT}" ALL DEPENDS "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}")
  add_dependencies(revng-all-binaries "early-linked-module-${OUTPUT}")
  install(FILES "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
    DESTINATION share/revng)

  # Enable the support for C exceptions to avoid optimizations that break
  # exception support when linking a module with isolated functions
  foreach(CONFIG ${SUPPORT_MODULES_CONFIGS})
    set(OUTPUT "support-${ARCH}-${CONFIG}.ll")
    add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
      DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/support.c"
      COMMAND "${CLANG}"
      ARGS "${CMAKE_CURRENT_SOURCE_DIR}/runtime/support.c"
           -O2
           -fexceptions
           -o "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
           -S -emit-llvm -g
           -DTARGET_${ARCH}
           -I"${CMAKE_CURRENT_SOURCE_DIR}/runtime"
           -I"${CMAKE_CURRENT_SOURCE_DIR}/include"
           ${SUPPORT_MODULES_CONFIG_${CONFIG}})
    add_custom_target("support-module-${OUTPUT}" ALL DEPENDS "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}")
    add_dependencies(revng-all-binaries "support-module-${OUTPUT}")
    install(FILES "${CMAKE_BINARY_DIR}/share/revng/${OUTPUT}"
      DESTINATION share/revng)
  endforeach()
endforeach()

add_definitions("-DINSTALL_PATH=\"${CMAKE_INSTALL_PREFIX}\"")
add_definitions("-DBUILD_PATH=\"${CMAKE_BINARY_DIR}\"")

#
# Compiler options
#

# Remove -rdynamic
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS)

# Basic compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wextra")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-exceptions")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Disable some warnings
add_flag_if_available("-Wno-unused-parameter")
add_flag_if_available("-Wno-unused-variable")
add_flag_if_available("-Wno-maybe-uninitialized")
add_flag_if_available("-Wno-init-list-lifetime")
add_flag_if_available("-Wno-ambiguous-reversed-operator")

# Add some extra warnings
add_flag_if_available("-Wstrict-aliasing")
add_flag_if_available("-fstrict-aliasing")

add_flag_if_available("-Wnon-virtual-dtor")
add_flag_if_available("-Wunreachable-code-break")
add_flag_if_available("-Winconsistent-missing-destructor-override")
add_flag_if_available("-Wnewline-eof")
add_flag_if_available("-Wmissing-prototypes")

add_definitions("-D_FILE_OFFSET_BITS=64")

CHECK_CXX_COMPILER_FLAG("-no-pie" COMPILER_SUPPORTS_NO_PIE)
if(COMPILER_SUPPORTS_NO_PIE)
  set(NO_PIE "-no-pie")
endif()

include(CheckIncludeFiles)
CHECK_INCLUDE_FILES(valgrind/callgrind.h HAVE_VALGRIND_CALLGRIND_H)
if(HAVE_VALGRIND_CALLGRIND_H)
add_definitions("-DHAVE_VALGRIND_CALLGRIND_H")
endif()

set(VERSION 0.0.0)

function(copy_to_build_and_install INSTALL_TYPE DESTINATION)
  foreach(INPUT_FILE ${ARGN})
    make_directory("${CMAKE_BINARY_DIR}/${DESTINATION}" )
    configure_file("${INPUT_FILE}" "${CMAKE_BINARY_DIR}/${DESTINATION}" COPYONLY)
    install("${INSTALL_TYPE}" "${INPUT_FILE}" DESTINATION "${DESTINATION}")
  endforeach()
endfunction()

#
# Support files (share/revng)
#
copy_to_build_and_install(FILES
  share/revng
  runtime/support.c
  runtime/support.h
  scripts/clang-format-style-file)

configure_file(runtime/early-linked.c "${CMAKE_BINARY_DIR}/share/revng/early-linked.c" COPYONLY)

# Custom command to create .clang-format file from check-revng-conventions
add_custom_command(OUTPUT "${CMAKE_BINARY_DIR}/share/revng/.clang-format"
  DEPENDS
    "${CMAKE_BINARY_DIR}/bin/check-revng-conventions"
    "${CMAKE_BINARY_DIR}/share/revng/clang-format-style-file"
  COMMAND "${CMAKE_BINARY_DIR}/bin/check-revng-conventions"
  ARGS
    --print-clang-format-config
    >
    "${CMAKE_BINARY_DIR}/share/revng/.clang-format"
)

add_custom_target(clang-format-dot-file ALL DEPENDS "${CMAKE_BINARY_DIR}/share/revng/.clang-format" )
install(FILES "${CMAKE_BINARY_DIR}/share/revng/.clang-format" DESTINATION share/revng)

#
# Executable scripts
#

# revng must not have COPYONLY
configure_file(scripts/revng "${CMAKE_BINARY_DIR}/bin/revng")
install(PROGRAMS "${CMAKE_BINARY_DIR}/bin/revng" DESTINATION bin)

copy_to_build_and_install(PROGRAMS
  bin
  "scripts/check-revng-conventions"
  "scripts/revng-merge-dynamic"
  )

copy_to_build_and_install(FILES
  bin/revng_merge_dynamic
  "scripts/revng_merge_dynamic/__init__.py"
  "scripts/revng_merge_dynamic/__main__.py"
  "scripts/revng_merge_dynamic/log.py"
  "scripts/revng_merge_dynamic/merge_dynamic.py"
  "scripts/revng_merge_dynamic/parsed_elf.py"
  "scripts/revng_merge_dynamic/util.py"
  )

copy_to_build_and_install(FILES
  bin/revng_dump_model
  "scripts/revng_dump_model/__init__.py"
  "scripts/revng_dump_model/__main__.py"
  "scripts/revng_dump_model/dump_model.py"
  )

#
# Export CMake targets
#
install(EXPORT revng NAMESPACE revng:: DESTINATION share/revng/cmake)

#
# Include other CMake files
#
add_subdirectory(include)
add_subdirectory(lib)
add_subdirectory(tools)
add_subdirectory(docs/)

include(${CMAKE_INSTALL_PREFIX}/share/revng/qa/cmake/revng-qa.cmake)

include(tests/Tests.cmake)
