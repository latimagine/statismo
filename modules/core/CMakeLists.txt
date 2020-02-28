# Create target

set(_target_include_dir_suffix /statismo/core)
set(_target_include_dir include/${_target_include_dir_suffix})

file(GLOB _target_headers
  ${_target_include_dir}/*.h ${_target_include_dir}/*.hxx ${_target_include_dir}/*.hpp ${_target_include_dir}/*.txx ${_target_include_dir}/*.tpp
)

add_library(statismo_core ${statismo_LIB_TYPE}
  src/ModelInfo.cxx
)

set(_deps
  ${HDF5_LIBRARIES}
)

if(UNIX)
  set(_deps
    ${_deps}
    pthread
  )
endif()

target_link_libraries(statismo_core PUBLIC ${_deps})
target_include_directories(statismo_core PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_compile_features(statismo_core PUBLIC cxx_std_17)
target_compile_options(statismo_core PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(statismo_core PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION ${statismo_LIB_VERSION}
  SOVERSION ${statismo_LIB_SOVERSION}
)

if(${BUILD_WITH_TIDY})
  set_target_properties(
   statismo_core PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}" 
  )
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
generate_export_header(
  statismo_core
  BASE_NAME STATISMO_CORE
  EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoCoreExport.h
  INCLUDE_GUARD_NAME __STATISMO_CORE_EXPORT_H_
  )

# Test

if(${BUILD_TESTS})
  add_subdirectory(tests)
endif()

# Install

install(TARGETS statismo_core
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

install(FILES ${_target_headers} ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoCoreExport.h
  DESTINATION ${INSTALL_INCLUDE_DIR}/${_target_include_dir_suffix}
)
