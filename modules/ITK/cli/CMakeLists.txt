# The idea here is to have an X.cxx and a X.md where X denotes the cli-command name.

include_directories(${LPO_INCLUDE_DIR})

# Exe build
set(_cli_files
  statismo-build-shape-model
  statismo-build-deformation-model
  statismo-build-gp-model
  statismo-sample
  statismo-reduce-model
  statismo-fit-surface
  statismo-fit-image
  statismo-posterior
  statismo-warp-image
)

foreach(_ex ${_cli_files})
  add_executable(${_ex} ${_ex}.cxx)
  target_link_libraries(${_ex} statismo_core statismo_ITK)
  install(TARGETS ${_ex} DESTINATION ${INSTALL_BIN_DIR})
  target_compile_options(${_ex} PRIVATE "${STATISMO_COMPILE_OPTIONS}")
  target_compile_definitions(${_ex} PRIVATE -D_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING)
  set_target_properties(${_ex} PROPERTIES FOLDER cli)

  if(${BUILD_WITH_TIDY})
    set_target_properties(
      ${_ex}  PROPERTIES
    CXX_CLANG_TIDY "${WITH_CLANG_TIDY}"
    )
  endif()
endforeach()

# Doc build

if(${BUILD_CLI_TOOLS_DOC})
  if(PANDOC)
    # configuring man page install dir
    if(WIN32 AND NOT CYGWIN)
      #put the directory with the manuals in the same directory as the binaries on windows
      set(_clidoc_install_dir ${INSTALL_BIN_DIR})
      set(_clidoc_output_dir "${CMAKE_BINARY_DIR}/doc-cli")
    else()
      # use cmake to find the man pages directory on *NIX
      include(GNUInstallDirs)
      if(CMAKE_INSTALL_MANDIR)
        set(_clidoc_install_dir ${CMAKE_INSTALL_MANDIR})
      else()
        set(_clidoc_install_dir ${INSTALL_BIN_DIR})
        message(WARNING "man install directory not found (man pages will NOT be installed properly)")
      endif()
        set(_clidoc_output_dir "${CMAKE_BINARY_DIR}/man8")
    endif()

    set(_clidoc_target_deps)
    set(_clidoc_target_sources)

    foreach(e ${_cli_files})
      set(CLIDOC_MD_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/${e}.md)
      if(WIN32 AND NOT CYGWIN)
        set(CLIDOC_OUT_FILE ${_clidoc_output_dir}/${e}.html)
      else()
        set(CLIDOC_OUT_FILE ${_clidoc_output_dir}/${e}.8)
      endif()

      add_custom_command(
        OUTPUT ${CLIDOC_OUT_FILE}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_clidoc_output_dir}
        # a '/' instead of a '\' in the input-file's path on windows leads to pandoc failing to automatically recoginze the file format -> we have specify it manually
        COMMAND ${PANDOC} -f markdown -s ${CLIDOC_MD_SOURCE} -o ${CLIDOC_OUT_FILE}
        MAIN_DEPENDENCY ${CLIDOC_MD_SOURCE}
     )

      list(APPEND _clidoc_target_deps ${CLIDOC_OUT_FILE})
      list(APPEND _clidoc_target_sources ${CLIDOC_MD_SOURCE})
    endforeach()

    # build the cli documentation by default
    add_custom_target(
      statismo_doc_cli ALL
      DEPENDS ${_clidoc_target_deps}
      SOURCES ${_clidoc_target_sources}
   )
    # install the entire cli documentation directory
    install(
      DIRECTORY ${_clidoc_output_dir}
      DESTINATION ${_clidoc_install_dir}
   )
  else()
    message(WARNING "pandoc not found (cli tools doc generation disabled)")
  endif()
endif()

# Test build

if(${BUILD_LONG_RUNNING_CLI_TESTS})
  add_subdirectory(tests)
endif()
