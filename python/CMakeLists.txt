configure_file(__init__.py.plain.in
               ${ecto_module_PYTHON_OUTPUT}/__init__.py
               @ONLY
)

install(FILES ${ecto_module_PYTHON_OUTPUT}/ecto_cells/__init__.py
        DESTINATION ${PYTHON_PACKAGES_PATH}/object_recognition_capture/ecto_cells
)
