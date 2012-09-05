find_package(object_recognition_core REQUIRED)

include_directories(SYSTEM ${object_recognition_core_INCLUDE_DIRS})

#deal with the capture cells
ectomodule(capture DESTINATION ${PROJECT_NAME}/ecto_cells
                   INSTALL
                   module.cpp
                   DeltaRT.cpp
)

link_ecto(capture
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
)
