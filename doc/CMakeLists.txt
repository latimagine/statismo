if(BUILD_DOCUMENTATION)
  find_package(LATEX)
  if(NOT LATEX_COMPILER)
    message(WARNING "LaTeX not found (formulas generation disabled)")
  endif()

  set(DOXYGEN_PROJECT_NAME ${PROJECT_NAME})
  set(DOXYGEN_PROJECT_NUMBER ${PROJECT_VERSION})
  set(DOXYGEN_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
  set(DOXYGEN_STRIP_FROM_PATH ${PROJECT_SOURCE_DIR})
  set(DOXYGEN_PROJECT_BRIEF "Statismo doxygen documentation")
  #set(DOXYGEN_IMAGE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/classdoc)
  set(DOXYGEN_GENERATE_HTML YES)
  set(DOXYGEN_WARN_IF_UNDOCUMENTED NO)
  set(DOXYGEN_CITE_BIB_FILES ${CMAKE_CURRENT_SOURCE_DIR}/dox/main.bib)
  set(DOXYGEN_EXCLUDE_PATTERNS ${PROJECT_SOURCE_DIR}/modules/ITK/cli/*)

  doxygen_add_docs(
      statismo_dox
      ${CMAKE_CURRENT_SOURCE_DIR}/dox/intro.dox ${PROJECT_SOURCE_DIR}/modules
      ${PROJECT_SOURCE_DIR}/data
      COMMENT "Generate doxygen doc"
      ALL
  )

  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/dox/statismofileformat.html
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/html/)
  
  file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/dox/images
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/html)

  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/html/
    DESTINATION ${INSTALL_DOC_DIR}/html
  )
endif()