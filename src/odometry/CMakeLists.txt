find_package(OpenCV REQUIRED)

# there is no odometry in 2.3
if (${OpenCV_VERSION} VERSION_LESS "2.4.0")
  return()
endif()

include_directories(${OpenCV_INCLUDE_DIRS})

# Build the demo from OpenCV
add_executable(rgbd rgbdodometry.cpp)

target_link_libraries(rgbd ${OpenCV_LIBS})

# Build the odometry ecto module
find_package(object_recognition_core REQUIRED)
find_package(ecto_opencv REQUIRED)

include_directories(SYSTEM ${object_recognition_core_INCLUDE_DIRS}
                           ${ecto_opencv_INCLUDE_DIRS}
)

ectomodule(odometry
           module.cpp
           odometry.cpp
)

link_ecto(odometry
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
          ${ecto_opencv_LIBRARIES}
)

install_ecto_module(odometry)
