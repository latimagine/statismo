# common options used by project CMakeLists.txt
# and superbuild

option(BUILD_DOCUMENTATION "Build doxygen documentation" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(BUILD_WITH_TIDY "Build with clang-tidy sanity check and code style" OFF)
option(ITK_SUPPORT "Build ITK module" ON)
option(VTK_SUPPORT "Build VTK module" ON)
option(ENABLE_RUNTIME_LOGS "Enable runtime logging" OFF)
include(${CMAKE_ROOT}/Modules/Documentation.cmake)
mark_as_advanced(BUILD_DOCUMENTATION)

include(CMakeDependentOption)
cmake_dependent_option(BUILD_WRAPPING "Build VTK module python wrapper" OFF
  "VTK_SUPPORT" OFF
)
cmake_dependent_option(BUILD_CLI_TOOLS "Build ITK based cli tools" ON
  "ITK_SUPPORT" OFF
)
cmake_dependent_option(BUILD_CLI_TOOLS_DOC "Build ITK based cli tools documentation" OFF
  "BUILD_CLI_TOOLS" OFF
)
cmake_dependent_option(BUILD_LONG_RUNNING_CLI_TESTS "Build long running ITK based cli tools tests" ON
  "BUILD_CLI_TOOLS;BUILD_TESTS" OFF
)

if(BUILD_WRAPPING)
  set(STATISMO_PYTHON_VERSION 3 CACHE STRING "Python version for VTK module python wrapper")
endif()
