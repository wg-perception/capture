cmake_minimum_required(VERSION 2.8)
project(object_recognition_capture)
find_package(catkin QUIET)
catkin_stack()
catkin_project(object_recognition_capture)

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)
find_package(object_recognition_core REQUIRED)

set_ecto_install_package_name(${PROJECT_NAME}/ecto_cells)

include_directories(include)

include_directories(SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${object_recognition_core_INCLUDE_DIRS}
)

catkin_python_setup()

add_subdirectory(python)
add_subdirectory(cells)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

if (ROS_ELECTRIC_FOUND)
  set(${PROJECT_NAME}_BIN_INSTALL_DIR bin)
else()
  set(${PROJECT_NAME}_BIN_INSTALL_DIR share/${PROJECT_NAME}/bin)
endif()

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/capture
                 ${PROJECT_SOURCE_DIR}/apps/orb_template.py
                 ${PROJECT_SOURCE_DIR}/apps/orb_track.py
                 ${PROJECT_SOURCE_DIR}/apps/upload
        DESTINATION ${${PROJECT_NAME}_BIN_INSTALL_DIR}
)

install(DIRECTORY apps/capture_board apps/bagscripts
        DESTINATION ${${PROJECT_NAME}_BIN_INSTALL_DIR}
        USE_SOURCE_PERMISSIONS
)

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
