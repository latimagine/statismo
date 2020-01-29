set(_target_tests
  itkStandardImageRepresenterTest.cxx
  itkStandardMeshRepresenterTest.cxx
)

create_test_sourcelist( 
    itkStatismoTest
    itkStatismoTest_driver.cxx
    ${_target_tests}
)

add_executable(itkStatismoTest itkStatismoTest_driver.cxx ${_target_tests})
target_link_libraries(itkStatismoTest statismo_ITK)

if(${BUILD_WITH_TIDY})
   set_target_properties(
    itkStatismoTest PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

foreach(_test ${_target_tests})
    get_filename_component(TNAME ${_test} NAME_WE)
    add_test(NAME ${TNAME} COMMAND itkStatismoTest ${TNAME} ${statismo_SOURCE_DIR}/data WORKING_DIRECTORY "${TEST_WORKING_DIR}")
endforeach()
