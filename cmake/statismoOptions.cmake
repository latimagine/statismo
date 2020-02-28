# common options used by project CMakeLists.txt
# and superbuild

option(BUILD_DOCUMENTATION "Build doxygen documentation" ON)
option(BUILD_EXAMPLES "Build examples" ON)
option(BUILD_TESTS "Build tests" ON)
option(BUILD_SHARED_LIBS "Build shared libs" ON)
option(BUILD_WITH_TIDY "Build with clang-tidy for more sanity" OFF)
option(ITK_SUPPORT "Build with ITK Support" ON)
option(VTK_SUPPORT "Build with VTK Support" ON)
include(${CMAKE_ROOT}/Modules/Documentation.cmake)
mark_as_advanced(BUILD_DOCUMENTATION)

include(CMakeDependentOption)
cmake_dependent_option(BUILD_WRAPPING "Build VTK python wrappers (experimental)" OFF
  "VTK_SUPPORT" OFF
)
cmake_dependent_option(BUILD_CLI_TOOLS "Build cli tools" ON
  "ITK_SUPPORT" OFF
)
cmake_dependent_option(BUILD_CLI_TOOLS_DOC "Build documentation for the command-line tools" OFF
  "BUILD_CLI_TOOLS" OFF
)
cmake_dependent_option(BUILD_LONG_RUNNING_CLI_TESTS "Run the cli examples (the execution of these tests can take some time)" ON
  "BUILD_CLI_TOOLS;BUILD_TESTS" OFF
)

if(BUILD_WRAPPING)
  set(STATISMO_PYTHON_VERSION 3 CACHE STRING "Python version for wrapping")
endif()
