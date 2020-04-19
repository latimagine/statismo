# Create target

set(_target_include_dir_suffix /statismo/ITK)
set(_target_include_dir include/${_target_include_dir_suffix})

file(GLOB _target_headers
  ${_target_include_dir}/*.h ${_target_include_dir}/*.hxx ${_target_include_dir}/*.hpp ${_target_include_dir}/*.txx ${_target_include_dir}/*.tpp
)

add_library(statismo_ITK ${statismo_LIB_TYPE}
  src/itkOutputWindowLogPolicies.cxx
  src/itkStatismoOutputWindow.cxx
)
target_link_libraries(statismo_ITK statismo_core ${ITK_LIBRARIES})
target_include_directories(
    statismo_ITK PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_compile_features(statismo_ITK PUBLIC cxx_std_17)
target_compile_options(statismo_ITK PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(statismo_ITK PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION ${statismo_LIB_VERSION}
  SOVERSION ${statismo_LIB_SOVERSION}
)
if(${BUILD_WITH_TIDY})
   set_target_properties(
    statismo_ITK PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
generate_export_header(
  statismo_ITK
  BASE_NAME STATISMO_ITK
  EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoITKExport.h
  INCLUDE_GUARD_NAME __STATISMO_ITK_EXPORT_H_
  )

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

install(FILES ${_target_headers} ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoITKExport.h
  DESTINATION ${INSTALL_INCLUDE_DIR}/${_target_include_dir_suffix}
)
