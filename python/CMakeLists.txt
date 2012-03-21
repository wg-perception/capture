#install targets for all things python
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/capture
  DESTINATION ${PYTHON_PACKAGES_PATH}
  COMPONENT main
  FILES_MATCHING PATTERN "*.py"
)

# we want the split version for buildspace as sources are in two directories
configure_file(__init__.py.split.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/__init__.py
               @ONLY
)
configure_file(__init__.py.plain.in
               ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
               @ONLY
)

# we actually want the plain one for install
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core
)
install(FILES ${python_LIBRARY_OUTPUT_DIRECTORY}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_core/ecto_cells
)
