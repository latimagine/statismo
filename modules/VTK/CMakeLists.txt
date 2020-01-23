# Create target
add_library(statismo_VTK ${statismo_LIB_TYPE}
  src/vtkStandardMeshRepresenter.cxx
  src/vtkUnstructuredGridRepresenter.cxx
)
target_link_libraries(statismo_VTK PUBLIC statismo_core ${VTK_LIBRARIES})
target_include_directories(statismo_VTK PUBLIC  
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/>
)
target_compile_features(statismo_VTK PUBLIC cxx_std_17)
target_compile_options(statismo_VTK PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(statismo_VTK PROPERTIES
  DEBUG_POSTFIX "-d"
  VERSION "${statismo_LIB_VERSION}"
  SOVERSION "${statismo_LIB_SOVERSION}"
)
if(${BUILD_WITH_TIDY})
   # Const-cast is used because, in VTK, few routines are const even if
   # the specific invariant of the class are not modified 
   set(WITH_CLANG_TIDY "${WITH_CLANG_TIDY},-cppcoreguidelines-pro-type-const-cast")
   set_target_properties(
    statismo_VTK PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

set(TARGET_INCLUDE_DIR_SUFFIX /statismo/VTK)
set(TARGET_INCLUDE_DIR include/${TARGET_INCLUDE_DIR_SUFFIX})

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

file(GLOB TARGET_HEADERS
  ${TARGET_INCLUDE_DIR}/*.h ${TARGET_INCLUDE_DIR}/*.hxx ${TARGET_INCLUDE_DIR}/*.hpp ${TARGET_INCLUDE_DIR}/*.txx ${TARGET_INCLUDE_DIR}/*.tpp
)
install(FILES ${TARGET_HEADERS}
  DESTINATION ${INSTALL_INCLUDE_DIR}/${TARGET_INCLUDE_DIR_SUFFIX}
)
