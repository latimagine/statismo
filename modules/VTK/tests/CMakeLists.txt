set(_target_tests
  PosteriorModelBuilderTest.cxx
  PCAModelBuilderWithSelfAdjointEigenSolverTest.cxx
  PCAModelBuilderTest.cxx
  vtkStandardImageRepresenterTest.cxx
  vtkStandardMeshRepresenterTest.cxx
)

create_test_sourcelist( 
    vtkStatismoTest
    vtkStatismoTest_driver.cxx
    ${_target_tests}
)

add_executable(vtkStatismoTest vtkStatismoTest_driver.cxx ${_target_tests} vtkTestHelper.h)
target_link_libraries(vtkStatismoTest statismo_VTK)

if(${BUILD_WITH_TIDY})
   set_target_properties(
    vtkStatismoTest PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

foreach(_test ${_VTKTests} )
  add_executable( ${_test} ${_test}.cxx )
  target_link_libraries( ${_test} statismo_VTK )
endforeach()

foreach(TST ${_target_tests})
    get_filename_component(TNAME ${TST} NAME_WE)
    add_test(NAME ${TNAME} COMMAND vtkStatismoTest ${TNAME} ${statismo_SOURCE_DIR}/data WORKING_DIRECTORY "${TEST_WORKING_DIR}")
endforeach()
