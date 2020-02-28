include_directories(${UNIT_TEST_INCLUDE_DIR})

set(_target_tests
  basicStatismoTest.cxx
  utilsStatismoTest.cxx
)

create_test_sourcelist( 
    coreStatismoTest
    coreStatismoTest_driver.cxx
    ${_target_tests}
)

add_executable(coreStatismoTest coreStatismoTest_driver.cxx ${_target_tests})
target_link_libraries(coreStatismoTest statismo_core)
target_compile_options(coreStatismoTest PRIVATE "${STATISMO_COMPILE_OPTIONS}")
set_target_properties(coreStatismoTest PROPERTIES FOLDER tests)

if(${BUILD_WITH_TIDY})
   set_target_properties(
    coreStatismoTest PROPERTIES
   CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
  )
endif()

foreach(_tst ${_target_tests})
    get_filename_component(TNAME ${_tst} NAME_WE)
    add_test(NAME ${TNAME} COMMAND coreStatismoTest ${TNAME})
    if (MSVC)
        set_tests_properties(${TNAME} PROPERTIES ENVIRONMENT "PATH=${STATISMO_ENV_PATH}")
    endif()
endforeach()