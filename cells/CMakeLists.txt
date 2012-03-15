include_directories(${CMAKE_CURRENT_SOURCE_DIR})

#deal with the capture cells
set_ecto_install_package_name(capture/ecto_cells)

ectomodule(capture
  module.cpp
  DeltaRT.cpp
  )

link_ecto(capture
    object_recognition_core_db
    opencv_core
    opencv_calib3d
)

install_ecto_module(capture)
