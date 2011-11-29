execute_process(COMMAND mkdir -p ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/capture)
execute_process(COMMAND ln -sf ${CMAKE_CURRENT_SOURCE_DIR}/capture/__init__.py
  WORKING_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/capture
  )

#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/capture
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
  FILES_MATCHING PATTERN "*.py"
)

