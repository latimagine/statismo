# Create target

set(_target_include_dir_suffix /statismo/VTK)
set(_target_include_dir include/${_target_include_dir_suffix})

file(GLOB _target_headers
  ${_target_include_dir}/*.h ${_target_include_dir}/*.hxx ${_target_include_dir}/*.hpp ${_target_include_dir}/*.txx ${_target_include_dir}/*.tpp
)

add_library(statismo_VTK ${statismo_LIB_TYPE}
  src/vtkOutputWindowLogPolicies.cxx
  src/vtkStandardMeshRepresenter.cxx
  src/vtkStatismoOutputWindow.cxx
  src/vtkUnstructuredGridRepresenter.cxx
)
target_link_libraries(statismo_VTK PUBLIC statismo_core ${VTK_LIBRARIES})
target_include_directories(statismo_VTK PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_compile_features(statismo_VTK PUBLIC cxx_std_17)
target_compile_options(statismo_VTK PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION ${statismo_LIB_VERSION}
  SOVERSION ${statismo_LIB_SOVERSION}
)
if(${BUILD_WITH_TIDY})
   set_target_properties(
    statismo_VTK PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

include(GenerateExportHeader)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_VISIBILITY_INLINES_HIDDEN 1)
generate_export_header(
  statismo_VTK
  BASE_NAME STATISMO_VTK
  EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoVTKExport.h
  INCLUDE_GUARD_NAME __STATISMO_VTK_EXPORT_H_
  )

# Test

if(${BUILD_TESTS})
  add_subdirectory(tests)
endif()

# Example

if(${BUILD_EXAMPLES})
  add_subdirectory(examples)
endif()

# Wrapping

if(${BUILD_WRAPPING})
  add_subdirectory(wrapping)
endif()

# Install

install(TARGETS statismo_VTK
  EXPORT statismoTargets
  RUNTIME DESTINATION ${INSTALL_BIN_DIR}
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)

install(FILES ${_target_headers} ${CMAKE_CURRENT_BINARY_DIR}/${_target_include_dir}/StatismoVTKExport.h
  DESTINATION ${INSTALL_INCLUDE_DIR}/${_target_include_dir_suffix}
)
