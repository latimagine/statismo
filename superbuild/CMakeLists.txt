cmake_minimum_required(VERSION 3.13.4)
project(Statismo-Superbuild LANGUAGES C CXX)

# ----------------------------------------
# Build type settings (https://blog.kitware.com/cmake-and-the-default-build-type/)
# ----------------------------------------

# Set a default build type if none was specified
set(_default_build_type "Release")
if(NOT BUILD_TYPE AND NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "setting build type to '${_default_build_type}' as none was specified.")
  set(BUILD_TYPE "${_default_build_type}" CACHE
      STRING "Choose the type of build" FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# ----------------------------------------
# Project options
# ----------------------------------------

include(${PROJECT_SOURCE_DIR}/../cmake/statismoOptions.cmake)

option(AUTOBUILD_STATISMO "Set to on build statismo during superbuild" ON)

include(CMakeDependentOption)
cmake_dependent_option(USE_SYSTEM_VTK "Use system libraries for VTK" OFF
  "VTK_SUPPORT" OFF
)
cmake_dependent_option(USE_SYSTEM_ITK "Use system libraries for ITK" OFF
  "ITK_SUPPORT" OFF
)
# USE_SYSTEM_HDF5 and USE_SYSTEM_EIGEN are only an option when
# USE_SYSTEM_ITK=OFF (no ITK support or manually set to OFF).
# When USE_SYSTEM_ITK=ON, USE_SYSTEM_HDF5 and USE_SYSTEM_EIGEN are
# immutable and set to ON because we don't want to mix an ITK system
# bound with its hdf5/eigen libraries and hdf5/eigen libraries downloaded
# through the superbuild
cmake_dependent_option(USE_SYSTEM_HDF5 "Use system libraries for hdf5" OFF
  "NOT USE_SYSTEM_ITK" ON
)
cmake_dependent_option(USE_SYSTEM_EIGEN "Use system libraries for eigen3" OFF
  "NOT USE_SYSTEM_ITK" ON
)
cmake_dependent_option(USE_ITK_EIGEN "Use eigen from itk system library" OFF
  "USE_SYSTEM_ITK" OFF
)
cmake_dependent_option(USE_ITK_HDF5 "Use hdf5 from itk system library" OFF
  "USE_SYSTEM_ITK" OFF
)

if (VTK_SUPPORT AND NOT USE_SYSTEM_VTK)
  set(VTK_EXTRA_OPTIONS "" CACHE STRING "Build VTK with extra options")
endif()

if (ITK_SUPPORT AND NOT USE_SYSTEM_ITK)
  set(ITK_EXTRA_OPTIONS "" CACHE STRING "Build ITK with extra options")
endif()
# ----------------------------------------
# Project settings
# ----------------------------------------
# set CMAKE_MODULE_PATH for cmake macro/function and modules
set(CMAKE_MODULE_PATH
  ${CMAKE_MODULE_PATH}
  ${CMAKE_CURRENT_SOURCE_DIR}/../cmake
)

if(APPLE)
  include(CMakeOSXVariables)
endif()

set(BUILD_TYPE ${CMAKE_BUILD_TYPE})
if(NOT INSTALL_DEPENDENCIES_DIR)
    set(INSTALL_DEPENDENCIES_DIR ${CMAKE_BINARY_DIR}/INSTALL CACHE STRING "Install directory for dependencies")
endif()
set(Statismo_DEPENDENCIES)

# ----------------------------------------
# Project dependencies
# ----------------------------------------

include(ExternalProject)

# This is important to handle VTK before ITK as 
# ITK can use VTK with its Glue module
if(VTK_SUPPORT)
  if(NOT USE_SYSTEM_VTK)
    include(External-VTK.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} VTK)
  else()
    find_package(VTK 8.0.0 REQUIRED)
  endif()
endif()

set(_use_itk_hdf5 OFF)
set(_use_itk_eigen OFF)

if(NOT ITK_SUPPORT)
  # If no ITK support, the opposite of USE_SYSTEM_(HDF5|Eigen) is to download
  # an external project
  if(NOT USE_SYSTEM_HDF5)
    include(External-HDF5.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} HDF5)
  else()
    find_package(HDF5 1.10.2 REQUIRED COMPONENTS C CXX)
  endif()
  if(NOT USE_SYSTEM_EIGEN)
    include(External-Eigen.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} Eigen3)
  else()
    find_package(Eigen3 3.3.5 REQUIRED)
  endif()
else()
  if(NOT USE_SYSTEM_ITK)
    # If no ITK support, the opposite of USE_SYSTEM_(HDF5|Eigen) is to 
    # use ITK internal libraries
    if(USE_SYSTEM_HDF5)
      find_package(HDF5 1.10.2 REQUIRED COMPONENTS C CXX)
    else()
      set(_use_itk_hdf5 ON)
    endif()
    if(USE_SYSTEM_EIGEN)
      find_package(Eigen3 3.3.5 REQUIRED)
    else()
      set(_use_itk_eigen ON)
    endif()
    include(External-ITK.cmake)
    set(Statismo_DEPENDENCIES ${Statismo_DEPENDENCIES} ITK)
  else()
    # This should never happen
    if (NOT USE_SYSTEM_HDF5 OR NOT USE_SYSTEM_EIGEN)
      message(FATAL_ERROR "eigen and hdf5 must be system if ITK is system")
    endif()

    if (USE_ITK_EIGEN)
      set(_use_itk_eigen ON)
    else()
      find_package(Eigen3 3.3.5 REQUIRED)
    endif()
    if (USE_ITK_HDF5)
      set(_use_itk_hdf5 ON)
    else()
      find_package(HDF5 1.10.2 REQUIRED COMPONENTS C CXX)
    endif()

    # This has to be included after hdf5 and eigen
    find_package(ITK 5.0.0 REQUIRED)
  endif()
endif()

set (_cmake_args 
  ${_ep_common_args}
  ${CMAKE_HDF5_LIBS}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
  -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
  -DBUILD_DOCUMENTATION:BOOL=${BUILD_DOCUMENTATION}
  -DBUILD_TESTS:BOOL=${BUILD_TESTS}
  -DBUILD_EXAMPLES:BOOL=${BUILD_EXAMPLES}
  -DBUILD_CLI_TOOLS:BOOL=${BUILD_CLI_TOOLS}
  -DBUILD_WRAPPING:BOOL=${BUILD_WRAPPING}
  -DSTATISMO_PYTHON_VERSION:STRING=${STATISMO_PYTHON_VERSION}
  -DBUILD_WITH_TIDY:BOOL=${BUILD_WITH_TIDY}
  -DBUILD_CLI_TOOLS_DOC:BOOL=${BUILD_CLI_TOOLS_DOC}
  -DBUILD_LONG_RUNNING_CLI_TESTS:BOOL=${BUILD_LONG_RUNNING_CLI_TESTS}
  -DITK_SUPPORT:BOOL=${ITK_SUPPORT}
  -DVTK_SUPPORT:BOOL=${VTK_SUPPORT}
  -DINSTALL_DEPENDENCIES_DIR:PATH=${INSTALL_DEPENDENCIES_DIR}
  -DITK_DIR:PATH=${ITK_DIR}
  -DVTK_DIR:PATH=${VTK_DIR}
  -DHDF5_DIR:PATH=${HDF5_DIR}
  -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  -DEigen3_DIR:PATH=${Eigen3_DIR}
  -DUSE_ITK_EIGEN:BOOL=${_use_itk_eigen}
  -DUSE_ITK_HDF5:BOOL=${_use_itk_hdf5}
)

if(AUTOBUILD_STATISMO)
  ExternalProject_Add(Statismo
    DEPENDS ${Statismo_DEPENDENCIES}
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
    BINARY_DIR Statismo-build
    CMAKE_GENERATOR ${_ep_cmake_gen}
    CMAKE_ARGS ${_cmake_args}
    INSTALL_COMMAND ""
  )
else()
  ExternalProject_Add(Statismo
    DEPENDS ${Statismo_DEPENDENCIES}
    DOWNLOAD_COMMAND ""
    UPDATE_COMMAND ""
    PATCH_COMMAND ""
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/..
    BINARY_DIR Statismo-build
    CMAKE_GENERATOR ${_ep_cmake_gen}
    CMAKE_ARGS ${_cmake_args}
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
  )
endif()
