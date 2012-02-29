execute_process(COMMAND mkdir -p ${python_LIBRARY_OUTPUT_DIRECTORY})
execute_process(COMMAND ln -sf ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
                WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}
)

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/capture
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
  FILES_MATCHING PATTERN "*.py"
)

