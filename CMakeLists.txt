cmake_minimum_required(VERSION 3.13.4)
project(statismo VERSION 0.12.0 LANGUAGES C CXX)

# ----------------------------------------
# Project options
# ----------------------------------------

include(${PROJECT_SOURCE_DIR}/cmake/statismoOptions.cmake)

# ----------------------------------------
# Project settings
# ----------------------------------------
set(STATISMO_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(statismo_LIB_VERSION ${PROJECT_VERSION})
set(statismo_LIB_SOVERSION ${PROJECT_VERSION_MAJOR})
set(statismo_LIB_TYPE SHARED)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Library type
if (NOT BUILD_SHARED_LIBS)
  set(statismo_LIB_TYPE STATIC)
endif()
set(STATISMO_BUILD_SHARED ${BUILD_SHARED_LIBS})

# Setup build locations.
if(NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
  set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
endif()

# set CMAKE_MODULE_PATH for cmake macro/function and modules
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake/Modules" ${CMAKE_MODULE_PATH})

if (MSVC)
  include(CMakeTargetPath)
endif()

# ----------------------------------------
# Compile options
# ----------------------------------------
if(MSVC AND MSVC_VERSION GREATER 1310)
    set(STATISMO_REQUIRED_CXX_FLAGS "/bigobj;")
    string(APPEND STATISMO_COMPILE_OPTIONS "${STATISMO_REQUIRED_CXX_FLAGS}")
endif()

# ----------------------------------------
# Install instructions
# ----------------------------------------

set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for binaries")
set(INSTALL_INCLUDE_DIR include CACHE PATH
  "Installation directory for header files")
set(INSTALL_DOC_DIR share/doc/statismo CACHE PATH
  "Installation directory for documentation")

if(WIN32 AND NOT CYGWIN)
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR lib/cmake/statismo)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")

# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if(NOT IS_ABSOLUTE "${${var}}")
    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  endif()
endforeach()

# Used to store the path to shared library under windows
set(SHARED_LIBRARY_PATHS)

# ----------------------------------------
# Sanity
# ----------------------------------------

# Formating
set(CLANG_FORMAT_BIN_NAME clang-format-6.0 clang-format)
set(CLANG_FORMAT_EXCLUDE_PATTERNS  "build*" ${PROJECT_BINARY_DIR})
find_package(ClangFormat)

# Extra compiler options for warnings
string(
    APPEND STATISMO_COMPILE_OPTIONS
    "$<IF:$<CXX_COMPILER_ID:MSVC>,"
        "/W4;$<$<CONFIG:RELEASE>:/O2>,"
        "-Wall;-Wextra;-Werror;"
            "$<$<CONFIG:RELEASE>:-O3>;"
    ">"
)

# Clang tidy

if (BUILD_WITH_TIDY)
  find_program(
    CLANG_TIDY_EXE
    NAMES "clang-tidy"
    DOC "Path to clang-tidy executable"
  )

  if(NOT CLANG_TIDY_EXE)
    message(FATAL_ERROR "clang-tidy not found")
  else()
    set(WITH_CLANG_TIDY "${CLANG_TIDY_EXE}")
    set(CMAKE_EXPORT_COMPILE_COMMANDS 1)
  endif()
endif()

# ----------------------------------------
# Tests configuration
# ----------------------------------------

if (BUILD_TESTS)
  set(UNIT_TEST_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/helpers/tests")
  enable_testing()

  if (UNIX)
    find_program(MEMORYCHECK_COMMAND valgrind)
    if(MEMORYCHECK_COMMAND)
      set(MEMORYCHECK_COMMAND_OPTIONS "--trace-children=yes --leak-check=full")
      file(TO_CMAKE_PATH "${PROJECT_SOURCE_DIR}/cmake/valgrind.supp" MEMORYCHECK_SUPPRESSIONS_FILE)
    endif()
  endif()
  include(CTest)
endif()

# ----------------------------------------
# Dependencies
# ----------------------------------------

# Pandoc
if(${BUILD_CLI_TOOLS} AND ${BUILD_CLI_TOOLS_DOC})
  find_program(PANDOC NAMES pandoc)
  mark_as_advanced(PANDOC)
endif()

# VTK (first)
if(${VTK_SUPPORT})
  set(STATISMO_REQUIRED_VTK_MODULES
    vtkCommonComputationalGeometry
    vtkCommonCore
    vtkCommonDataModel
    vtkCommonExecutionModel
    vtkCommonMath
    vtkCommonMisc
    vtkCommonSystem
    vtkCommonTransforms
    vtkFiltersCore
    vtkFiltersGeneral
    vtkIOCore
    vtkIOGeometry
    vtkIOLegacy
    vtkIOXML
    vtkIOXMLParser
  )
  if(${BUILD_WRAPPING})
    list(APPEND STATISMO_REQUIRED_VTK_MODULES vtkWrappingPythonCore vtkWrappingTools)
  endif()

  list(FIND VTK_MODULES_ENABLED "vtkIOMPIParallel" HasOMPI)
  if(NOT HasOMPI EQUAL -1)
    list(APPEND STATISMO_REQUIRED_VTK_MODULES vtkIOMPIParallel)
  endif()
  # If VTK_DIR has been transferred, it will find it with first priority
  find_package(VTK 8.0.0 PATHS ${VTK_DIR} COMPONENTS ${STATISMO_REQUIRED_VTK_MODULES} NO_DEFAULT_PATH)
  if(NOT VTK_FOUND)
    find_package(VTK 8.0.0 REQUIRED COMPONENTS ${STATISMO_REQUIRED_VTK_MODULES})
  endif()
  include(${VTK_USE_FILE})
  if(MSVC)
    get_shared_locations(_vtk_libs_loc "${VTK_LIBRARIES}")
    set(SHARED_LIBRARY_PATHS "${SHARED_LIBRARY_PATHS};${_vtk_libs_loc}")
  endif()
endif()

# ITK (This has to be included first in case internal hdf5/eigen is used)
if(ITK_SUPPORT)
  # If ITK_DIR has been transfered, it will find it with first priority
  find_package(ITK 5.0.0 PATHS ${ITK_DIR} NO_DEFAULT_PATH)
  if(NOT ITK_FOUND)
    find_package(ITK 5.0.0 REQUIRED)
  endif()
  set(ITK_VERSION ${ITK_VERSION_MAJOR}.${ITK_VERSION_MINOR})
  include(${ITK_USE_FILE})
  if(MSVC)
    get_shared_locations(_itk_libs_loc "${ITK_LIBRARIES}")
    set(SHARED_LIBRARY_PATHS "${SHARED_LIBRARY_PATHS};${_itk_libs_loc}")
  endif()
endif()

# Eigen
if(ITK_SUPPORT AND USE_ITK_EIGEN)
  find_path(ITK_INCLUDE_WITH_EIGEN NAMES itkeigen/Eigen/Eigen PATHS ${ITK_INCLUDE_DIRS})
  set(EIGEN3_INCLUDE_DIR ${ITK_INCLUDE_WITH_EIGEN}/itkeigen)
else()
  # If EIGEN3_INCLUDE_DIR has been transferred, it will find it with first priority
  # Force find_package to MODULE to use the provided FindEigen3
  find_package(Eigen3 3.3.5 MODULE)
  if (NOT Eigen3_FOUND)
   find_package(Eigen3 3.3.5 REQUIRED)
  endif()
endif()
add_definitions(-DEIGEN_PERMANENTLY_DISABLE_STUPID_WARNINGS)
include_directories(${EIGEN3_INCLUDE_DIR})
# We have to use another variable name as ITK could modify this
set(CONFIG_EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})

# hdf5
if(ITK_SUPPORT AND USE_ITK_HDF5)
  find_path(ITK_INCLUDE_WITH_HDF5 NAMES itkhdf5/hdf5.h PATHS ${ITK_INCLUDE_DIRS})
  set(ITK_HDF5_DIR ${ITK_INCLUDE_WITH_HDF5}/itkhdf5)
  set(HDF5_INCLUDE_DIRS ${ITK_HDF5_DIR} ${ITK_HDF5_DIR}/cpp)
  set(HDF5_LIBRARIES ${ITKHDF5_LIBRARIES})
  include_directories(${HDF5_INCLUDE_DIRS})
  set(CONFIG_HDF5_INCLUDE_DIRS ${HDF5_INCLUDE_DIRS})
  set(CONFIG_HDF5_LIBRARIES ${HDF5_LIBRARIES})
else()
  # find hdf5
  if(BUILD_SHARED_LIBS)
    set(HDF5_COMPONENT_TYPE shared)
    set(HDF5_USE_STATIC_LIBRARIES OFF)
  else()
    set(HDF5_COMPONENT_TYPE static)
    set(HDF5_USE_STATIC_LIBRARIES ON)
  endif()
  # If HDF5_DIR has been transferred, it will find it with first priority
  find_package(HDF5 1.10.2 CONFIG PATHS ${HDF5_DIR} COMPONENTS C CXX ${HDF5_COMPONENT_TYPE} NO_DEFAULT_PATH)
  if (TARGET HDF5::HDF5) # Conan target
    set(HDF5_LIBRARIES HDF5::HDF5)
  else()
    if (NOT HDF5_${HDF5_COMPONENT_TYPE}_C_FOUND)
      # In case static/shared components does not exist
      find_package(HDF5 1.10.2 CONFIG PATHS ${HDF5_DIR} COMPONENTS C CXX NO_DEFAULT_PATH)
    endif()
    if (NOT HDF5_${HDF5_COMPONENT_TYPE}_C_FOUND)
      find_package(HDF5 1.10.2 REQUIRED COMPONENTS C CXX)
    endif()
    if(HDF5_${HDF5_COMPONENT_TYPE}_C_FOUND AND HDF5_${HDF5_COMPONENT_TYPE}_CXX_FOUND)
      # handle both case where hdf5 export targets have a namespace or not
      if (TARGET hdf5::hdf5-${HDF5_COMPONENT_TYPE})
        set(HDF5_LIBRARIES hdf5::hdf5-${HDF5_COMPONENT_TYPE} hdf5::hdf5_cpp-${HDF5_COMPONENT_TYPE})
      else()
        set(HDF5_LIBRARIES hdf5-${HDF5_COMPONENT_TYPE} hdf5_cpp-${HDF5_COMPONENT_TYPE})
      endif()
    else()
      # Package is imported the cmake way
      set(HDF5_LIBRARIES ${HDF5_C_LIBRARIES} ${HDF5_CXX_LIBRARIES})
    endif()
  endif()
  include_directories(${HDF5_INCLUDE_DIR})
  set(CONFIG_HDF5_DIR ${HDF5_DIR})
  if (MSVC)
    get_shared_locations(_hdf5_libs_loc "${HDF5_LIBRARIES}")
    set(SHARED_LIBRARY_PATHS "${SHARED_LIBRARY_PATHS};${_hdf5_libs_loc}")
  endif()
endif()

set(LPO_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/contrib/boost_light")

if (BUILD_WRAPPING)
  find_package(PythonLibs ${STATISMO_PYTHON_VERSION} REQUIRED)
  find_package(PythonInterp ${STATISMO_PYTHON_VERSION} REQUIRED)
  include_directories(${PYTHON_INCLUDE_PATH})
endif()

if (MSVC)
    set(_env_path "${SHARED_LIBRARY_PATHS};$ENV{PATH}")
    STRING(REPLACE ";" "\\;" STATISMO_ENV_PATH "${_env_path}")
endif()

# ----------------------------------------
# Process subdirectories
# ----------------------------------------

add_subdirectory(modules)
add_subdirectory(doc)

# ----------------------------------------
# Targets export generic settings
# ----------------------------------------

# Version file generation
include(CMakePackageConfigHelpers)

# ----------------------------------------
# Build-tree targets export
# ----------------------------------------

# Add all targets to the build-tree export set
set(_targets statismo_core)

if(VTK_SUPPORT)
  set(_targets ${_targets} statismo_VTK)
endif()

if(ITK_SUPPORT)
  set(_targets ${_targets} statismo_ITK)
endif()

export(TARGETS ${_targets}
  FILE "${statismo_BINARY_DIR}/statismoTargets.cmake"
)

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE statismo)

set(statismo_LIBRARIES statismo::statismo_core)

if(VTK_SUPPORT)
  set(statismo_LIBRARIES ${statismo_LIBRARIES} statismo::statismo_VTK)
endif()

if(ITK_SUPPORT)
  set(statismo_LIBRARIES ${statismo_LIBRARIES} statismo::statismo_ITK)
endif()

write_basic_package_version_file(
  ${statismo_BINARY_DIR}/statismoConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)

configure_package_config_file(
  ${statismo_SOURCE_DIR}/cmake/statismoConfig.cmake.in
  ${statismo_BINARY_DIR}/statismoConfig.cmake
  INSTALL_DESTINATION .
)

configure_package_config_file(
  ${statismo_SOURCE_DIR}/cmake/UseStatismo.cmake.in
  ${statismo_BINARY_DIR}/UseStatismo.cmake
  INSTALL_DESTINATION .
)

# ----------------------------------------
# Install tree targets export
# ----------------------------------------
configure_package_config_file(
  ${statismo_SOURCE_DIR}/cmake/statismoConfig.cmake.in
  ${statismo_BINARY_DIR}/cmake/statismoConfig.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
)

configure_package_config_file(
  ${statismo_SOURCE_DIR}/cmake/UseStatismo.cmake.in
  ${statismo_BINARY_DIR}/cmake/UseStatismo.cmake
  INSTALL_DESTINATION ${INSTALL_CMAKE_DIR}
)

install(FILES
    ${statismo_BINARY_DIR}/statismoConfig.cmake
    ${statismo_BINARY_DIR}/statismoConfigVersion.cmake
    ${statismo_BINARY_DIR}/UseStatismo.cmake
   DESTINATION ${INSTALL_CMAKE_DIR} COMPONENT dev
)
install(EXPORT statismoTargets NAMESPACE statismo::
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev
)
install(FILES README.md LICENSE CHANGELOG.md
  DESTINATION ${INSTALL_DOC_DIR}
)
