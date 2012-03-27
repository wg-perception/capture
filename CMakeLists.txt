cmake_minimum_required(VERSION 2.8)
project(capture)
catkin_stack()
catkin_project(capture
               INCLUDE_DIRS capture
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)
find_package(object_recognition_core REQUIRED)

set_ecto_install_package_name(capture)

include_directories(include)

include_directories(SYSTEM
    ${EIGEN_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${object_recognition_core_INCLUDE_DIRS}
)

if(NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

catkin_python_setup()
add_subdirectory(python)
add_subdirectory(cells)

option(BUILD_TEST "Build tests" ON)
if(BUILD_TEST)
  enable_testing()
  add_subdirectory(test)
endif()

install(DIRECTORY apps/
        DESTINATION bin
        USE_SOURCE_PERMISSIONS
        PATTERN "*.bag" EXCLUDE
        PATTERN "*.pyc" EXCLUDE
        )

#catkin_sphinx(${CMAKE_CURRENT_SOURCE_DIR}/doc doc)
