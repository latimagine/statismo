# Create target

set(_target_include_dir_suffix /statismo/ITK)
set(_target_include_dir include/${_target_include_dir_suffix})

file(GLOB _target_headers
  ${_target_include_dir}/*.h ${_target_include_dir}/*.hxx ${_target_include_dir}/*.hpp ${_target_include_dir}/*.txx ${_target_include_dir}/*.tpp
)

add_library(statismo_ITK INTERFACE)
target_include_directories(
    statismo_ITK INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_link_libraries(statismo_ITK INTERFACE statismo_core ${ITK_LIBRARIES})
target_compile_features(statismo_ITK INTERFACE cxx_std_17)

# Test

if(${BUILD_TESTS})
  add_subdirectory(tests)
endif()

# Example

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

# Tools

if(${BUILD_CLI_TOOLS})
  add_subdirectory(cli)
endif()

# Install

install(TARGETS statismo_ITK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

install(FILES ${_target_headers}
  DESTINATION ${INSTALL_INCLUDE_DIR}/${_target_include_dir_suffix}
)
