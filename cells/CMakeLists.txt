#deal with the capture cells
ectomodule(capture DESTINATION ${PROJECT_NAME}/ecto_cells
                   INSTALL
                   DeltaRT.cpp
                   FeatureFinder.cpp
                   MaskDisplay.cpp
                   module.cpp
                   PlaneFilter.cpp
)

link_ecto(capture
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBRARIES}
)
