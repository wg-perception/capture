cmake_minimum_required(VERSION 2.8)
project(object_recognition_capture)

find_package(catkin)

if (${catkin_VERSION} VERSION_GREATER "0.5.28")
find_package(catkin REQUIRED ecto object_recognition_core)
catkin_package()
else()
find_package(ROS REQUIRED ecto object_recognition_core)
catkin_stack()
catkin_project(object_recognition_capture)
set(catkin_LIBRARIES ${ROS_LIBRARIES})
set(catkin_INCLUDE_DIRS ${ROS_INCLUDE_DIRS})
set(CATKIN_DEVEL_PREFIX ${CMAKE_BINARY_DIR})
set(CATKIN_PACKAGE_BIN_DESTINATION share/${PROJECT_NAME}/bin)
set(CATKIN_PACKAGE_LIB_DESTINATION lib)
set(CATKIN_PACKAGE_PYTHON_BUILD_DESTINATION gen/py/${PROJECT_NAME}/)
set(CATKIN_PACKAGE_PYTHON_DESTINATION ${INSTALLED_PYTHONPATH}/${PROJECT_NAME}/)
install(FILES ${PROJECT_SOURCE_DIR}/manifest.xml
        DESTINATION share/${PROJECT_NAME}
)
endif()

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

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

# install the applications
install(PROGRAMS ${PROJECT_SOURCE_DIR}/apps/capture
                 ${PROJECT_SOURCE_DIR}/apps/orb_template.py
                 ${PROJECT_SOURCE_DIR}/apps/orb_track.py
                 ${PROJECT_SOURCE_DIR}/apps/upload
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
 
install(DIRECTORY apps/capture_board apps/bagscripts
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        USE_SOURCE_PERMISSIONS
)

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
