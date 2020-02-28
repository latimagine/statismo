cmake_policy(SET CMP0078 NEW)

if(${CMAKE_VERSION} VERSION_GREATER 3.14.0)
  cmake_policy(SET CMP0086 NEW)
endif()

# Give us path to exe SWIG_EXECUTABLE and dir SWIG_DIR
find_package(SWIG REQUIRED)
include(UseSWIG)

# Pre build
set(NUMPY_INCLUDE_DIR "" CACHE STRING "Numpy include path")
mark_as_advanced(NUMPY_INCLUDE_DIR)

# If not set, try to find it automatically
if ("${NUMPY_INCLUDE_DIR}" STREQUAL "")
  execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())"
    RESULTS_VARIABLE _numpy_search_res
    ERROR_VARIABLE _numpy_search_err
    OUTPUT_VARIABLE _numpy_search_out
    #COMMAND_ECHO STDOUT
    OUTPUT_STRIP_TRAILING_WHITESPACE # Important to fix format issue in flags.make
  )
  set(NUMPY_INCLUDE_DIR ${_numpy_search_out})
endif()

if ("${NUMPY_INCLUDE_DIR}" STREQUAL "")
  message(FATAL_ERROR "Include dir for numpy not found. You can set it manually with NUMPY_INCLUDE_DIR")
endif()

# Target configuration
set_source_files_properties(statismo.i PROPERTIES CPLUSPLUS ON SWIG_FLAGS "-includeall")

swig_add_library(statismovtk
 TYPE USE_BUILD_SHARED_LIBS
 LANGUAGE python
 SOURCES statismo.i)

swig_link_libraries(statismovtk statismo_core
  statismo_VTK
  ${PYTHON_LIBRARIES}
  ${VTK_LIBRARIES}
  ${HDF5_LIBRARIES})

target_include_directories(statismovtk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<BUILD_INTERFACE:${NUMPY_INCLUDE_DIR}>
)
target_compile_features(statismovtk PUBLIC cxx_std_17)
target_compile_definitions(statismovtk PUBLIC -DNPY_NO_DEPRECATED_API=NPY_1_7_API_VERSION)

# Remove the warnings due to register keyword in python 2 api
target_compile_options(statismovtk PRIVATE
     $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>,$<CXX_COMPILER_ID:GNU>>:
          -Wno-register>
)

set_target_properties(statismovtk PROPERTIES
  VERSION ${statismo_LIB_VERSION}
  SOVERSION ${statismo_LIB_SOVERSION}
)

# Tests
set(STATISMO_DATA_DIR "${statismo_SOURCE_DIR}/data/")
set(STATISMOVTK_TEST_SCRIPTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/tests/statismoTests")
set(STATISMOVTK_MODULE_DIR $<TARGET_FILE_DIR:statismovtk>)

if(MSVC)
    set(_script_ext "bat")
else()
    set(_script_ext "sh")
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/statismoTests/runVTKPythonTests.${_script_ext}.in ${PROJECT_BINARY_DIR}/runVTKPythonTests.${_script_ext}.gen @ONLY)
file(GENERATE OUTPUT ${PROJECT_BINARY_DIR}/runVTKPythonTests$<CONFIG>.${_script_ext} INPUT ${PROJECT_BINARY_DIR}/runVTKPythonTests.${_script_ext}.gen)

# Install
set(_python_install_dir ${INSTALL_LIB_DIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)
install(TARGETS statismovtk
    RUNTIME DESTINATION ${_python_install_dir}
    LIBRARY DESTINATION ${_python_install_dir}
    ARCHIVE DESTINATION ${_python_install_dir}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/statismovtk.py DESTINATION ${_python_install_dir})
