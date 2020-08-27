message(STATUS "External project: VTK")

set(_vtkOptions ${VTK_EXTRA_OPTIONS})

ExternalProject_Add(VTK
  GIT_REPOSITORY https://github.com/Kitware/VTK.git
  GIT_TAG v9.0.1 # If you modify this, update the VTK_DIR at the end of the file
  SOURCE_DIR VTK
  BINARY_DIR VTK-build
  UPDATE_COMMAND ""
  PATCH_COMMAND ""
  CMAKE_GENERATOR ${_ep_cmake_gen}
  CMAKE_ARGS
    ${_ep_common_args}
    ${_vtkOptions}
    -DBUILD_EXAMPLES:BOOL=OFF
    -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
    -DBUILD_TESTING:BOOL=OFF
    -DCMAKE_BUILD_TYPE:STRING=${BUILD_TYPE}
    -DVTK_BUILD_ALL_MODULES:BOOL=OFF
    -DVTK_USE_SYSTEM_HDF5:BOOL=OFF
    -DVTK_WRAP_PYTHON:BOOL=${BUILD_WRAPPING}
    -DVTK_PYTHON_VERSION:STRING=${STATISMO_PYTHON_VERSION}
    -DCMAKE_INSTALL_PREFIX:PATH=${INSTALL_DEPENDENCIES_DIR}
)

# This is passed to Statismo so it is able to find it in priority
set(VTK_DIR ${INSTALL_DEPENDENCIES_DIR}/lib/cmake/vtk-9.0/)
