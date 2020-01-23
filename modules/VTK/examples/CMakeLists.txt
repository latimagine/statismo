set(TARGET_EXAMPLES
  vtkBasicSamplingExample
  vtkBuildConditionalModelExample
  vtkBuildGaussianProcessShapeModelExample
  vtkBuildIntensityModelExample
  vtkBuildPosteriorModelExample
  vtkBuildShapeModelExample
  vtkCrossValidationExample
  vtkReduceModelVarianceExample
  vtkSpatiallyVaryingGPModelExample
)

foreach(e ${TARGET_EXAMPLES})
  add_executable(${e} ${e}.cxx)
  target_link_libraries(${e} statismo_VTK statismo_core ${VTK_LIBRARIES})
  target_compile_options(${e} PRIVATE "${STATISMO_COMPILE_OPTIONS}")

  if(${BUILD_WITH_TIDY})
    set_target_properties(
      ${e}  PROPERTIES
    CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
    )
  endif()
endforeach()

