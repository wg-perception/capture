cmake_minimum_required(VERSION 2.8)
project(object_recognition_capture)

find_package(catkin)

find_package(catkin REQUIRED ecto object_recognition_core)
catkin_package()

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(${catkin_INCLUDE_DIRS} include)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS}
                           ${OpenCV_INCLUDE_DIRS}
)

catkin_python_setup()

add_subdirectory(python)
add_subdirectory(cells)

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/capture
                 ${PROJECT_SOURCE_DIR}/apps/orb_template
                 ${PROJECT_SOURCE_DIR}/apps/orb_track
                 ${PROJECT_SOURCE_DIR}/apps/upload
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY apps/capture_board apps/bagscripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

ecto_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc/source)
