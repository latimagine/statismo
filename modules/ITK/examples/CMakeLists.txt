set(_target_examples
  itkBuildGaussianProcessDeformationModel
  itkBuildShapeModel_75pcvar
  itkLandmarkConstrainedShapeModelFitting
  itkShapeModelFitting
  itkBuildDeformationModel
  itkBuildShapeModel
  itkDeformationModelFitting
  itkLowRankGaussianProcessImageToImageRegistration
  itkSimpleGaussianProcessImageToImageRegistration
)

foreach(_ex ${_target_examples})
  add_executable(${_ex} ${_ex}.cxx)
  target_link_libraries(${_ex} statismo_ITK statismo_core)
  target_compile_options(${_ex} PRIVATE "${STATISMO_COMPILE_OPTIONS}")

  if(${BUILD_WITH_TIDY})
    set_target_properties(
      ${_ex}  PROPERTIES
    CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
    )
  endif()
endforeach()