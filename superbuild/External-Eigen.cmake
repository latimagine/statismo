message(STATUS "External project: Eigen")

set(Eigen3_VERSION "3.3.7")

ExternalProject_Add(Eigen3
  SOURCE_DIR ${CMAKE_BINARY_DIR}/Eigen3
  BINARY_DIR ${CMAKE_BINARY_DIR}/Eigen3-build
  URL "https://gitlab.com/libeigen/eigen/-/archive/${Eigen3_VERSION}/eigen-${Eigen3_VERSION}.tar.gz"
  CMAKE_ARGS
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DEPENDENCIES_DIR}
  INSTALL_DIR ${INSTALL_DEPENDENCIES_DIR}
)

set(EIGEN3_INCLUDE_DIR ${INSTALL_DEPENDENCIES_DIR}/include/eigen3)