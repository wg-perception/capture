cmake_minimum_required(VERSION 2.8)
project(capture)
find_package(catkin QUIET)
if (catkin_FOUND)
  catkin_stack()
  catkin_project(capture
                 INCLUDE_DIRS capture
  )
endif()

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)

add_subdirectory(src)

# if we are on ROS
if (catkin_FOUND)
  find_package(Eigen REQUIRED)
  find_package(ecto REQUIRED)
  find_package(object_recognition_core REQUIRED)

  set_ecto_install_package_name(capture)
endif()

include_directories(include)

include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${object_recognition_core_INCLUDE_DIRS}
)

if (catkin_FOUND)
  catkin_python_setup()
  add_subdirectory(python)
  add_subdirectory(cells)

  option(BUILD_TEST "Build tests" ON)
  if(BUILD_TEST)
    enable_testing()
    add_subdirectory(test)
  endif()
endif()

install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
)

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
