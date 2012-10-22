cmake_minimum_required(VERSION 2.8)
project(object_recognition_capture)
find_package(catkin QUIET)
if (${catkin_VERSION} VERSION_GREATER "0.5.28")
catkin_package()
else()
catkin_stack()
catkin_project(object_recognition_capture)
endif()

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)
find_package(object_recognition_core REQUIRED)

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

if (ROS_FUERTE_FOUND)
  set(${PROJECT_NAME}_BIN_INSTALL_DIR share/${PROJECT_NAME}/bin)
endif()

# install the applications
if (ROS_GROOVY_OR_ABOVE_FOUND)
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
else()
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
install(FILES ${PROJECT_SOURCE_DIR}/manifest.xml
        DESTINATION share/${PROJECT_NAME}
)
endif()

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
