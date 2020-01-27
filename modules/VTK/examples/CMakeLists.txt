set(_target_examples
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

foreach(_ex ${_target_examples})
  add_executable(${_ex} ${_ex}.cxx)
  target_link_libraries(${_ex} statismo_VTK statismo_core ${VTK_LIBRARIES})
  target_compile_options(${_ex} PRIVATE "${STATISMO_COMPILE_OPTIONS}")

  if(${BUILD_WITH_TIDY})
    set_target_properties(
      ${_ex}  PROPERTIES
    CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
    )
  endif()
endforeach()

