find_package(object_recognition_core REQUIRED)

include_directories(SYSTEM ${object_recognition_core_INCLUDE_DIRS})

#deal with the capture cells
set_ecto_install_package_name(capture/ecto_cells)

ectomodule(capture
  module.cpp
  DeltaRT.cpp
  )

link_ecto(capture
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
)

install_ecto_module(capture)
