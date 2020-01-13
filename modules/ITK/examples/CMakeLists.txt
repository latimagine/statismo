set(TARGET_EXAMPLES
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

foreach(e ${TARGET_EXAMPLES})
  add_executable(${e} ${e}.cxx)
  target_link_libraries(${e} statismo_ITK statismo_core)
  target_compile_options(${e} PRIVATE "${STATISMO_COMPILE_OPTIONS}")

  if(${BUILD_WITH_TIDY})
    set_target_properties(
      ${e}  PROPERTIES
    CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
    )
  endif()
endforeach()