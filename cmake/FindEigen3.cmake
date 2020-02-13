# - Try to find Eigen3 lib
#
# This module supports requiring a minimum version, e.g. you can do
#   find_package(Eigen3 3.1.2)
# to require version 3.1.2 or newer of Eigen3.
#
# Once done this will define
#
#  EIGEN3_FOUND - system has eigen lib with correct version
#  EIGEN3_INCLUDE_DIR - the eigen include directory
#  EIGEN3_VERSION - eigen version

# Copyright (c) 2006, 2007 Montel Laurent, <montel@kde.org>
# Copyright (c) 2008, 2009 Gael Guennebaud, <g.gael@free.fr>
# Copyright (c) 2009 Benoit Jacob <jacob.benoit.1@gmail.com>
# Redistribution and use is allowed according to the terms of the 2-clause BSD license.

macro(_eigen3_set_version)
  file(READ "${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" _eigen3_version_header)

  string(REGEX MATCH "define[ \t]+EIGEN_WORLD_VERSION[ \t]+([0-9]+)" _eigen3_world_version_match "${_eigen3_version_header}")
  set(EIGEN3_VERSION_MAJOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MAJOR_VERSION[ \t]+([0-9]+)" _eigen3_major_version_match "${_eigen3_version_header}")
  set(EIGEN3_VERSION_MINOR "${CMAKE_MATCH_1}")
  string(REGEX MATCH "define[ \t]+EIGEN_MINOR_VERSION[ \t]+([0-9]+)" _eigen3_minor_version_match "${_eigen3_version_header}")
  set(EIGEN3_VERSION_PATCH "${CMAKE_MATCH_1}")

  set(EIGEN3_VERSION ${EIGEN3_VERSION_MAJOR}.${EIGEN3_VERSION_MINOR}.${EIGEN3_VERSION_PATCH})
endmacro()

if(Eigen3_DIR)
  find_package(Eigen3 CONFIG PATHS ${Eigen3_DIR} NO_DEFAULt_PATH)
  set(EIGEN3_VERSION ${EIGEN3_VERSION_STRING})
endif()

if(NOT EIGEN3_INCLUDE_DIR)
  find_path(EIGEN3_INCLUDE_DIR NAMES signature_of_eigen3_matrix_library
      PATHS
      ${CMAKE_INSTALL_PREFIX}/include
      ${KDE4_INCLUDE_DIR}
      PATH_SUFFIXES eigen3 eigen
    )
endif()

if (EIGEN3_INCLUDE_DIR AND NOT EIGEN3_VERSION)
  _eigen3_set_version()
endif()

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(
  Eigen3 
  REQUIRED_VARS EIGEN3_INCLUDE_DIR EIGEN3_VERSION
  VERSION_VAR EIGEN3_VERSION)

mark_as_advanced(EIGEN3_INCLUDE_DIR)

