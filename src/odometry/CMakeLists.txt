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

include_directories(SYSTEM ${object_recognition_core_INCLUDE_DIRS})

set_ecto_install_package_name(${PROJECT_NAME}/ecto_cells)

ectomodule(odometry
           module.cpp
           odometry.cpp
)

link_ecto(odometry
          ${object_recognition_core_LIBRARIES}
          ${OpenCV_LIBS}
          ${OpenCV_LIBRARIES}
)

install_ecto_module(odometry)
