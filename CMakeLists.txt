cmake_minimum_required(VERSION 2.8)
project(capture)
set(${PROJECT_NAME}_MAJOR_VERSION 0)
set(${PROJECT_NAME}_MINOR_VERSION 1)
set(${PROJECT_NAME}_PATCH_VERSION 0)
set(${PROJECT_NAME}_SOVERSION 
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}
)
set(${PROJECT_NAME}_VERSION 
  ${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION}
)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_definitions("-Wall -Werror -Wl,--no-undefined ")

find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)
find_package(ecto REQUIRED)
find_package(object_recognition REQUIRED)

ecto_python_env_gen(${CMAKE_LIBRARY_OUTPUT_DIRECTORY} ${CMAKE_CURRENT_SOURCE_DIR}/python)
include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${object_recognition_INCLUDE_DIRS}
  )

link_directories(${object_recognition_LIBRARY_DIRS})

add_subdirectory(python)
#add_subdirectory(src)
add_subdirectory(cells)
add_subdirectory(doc)

#option(BUILD_TEST "Build tests" ON)
#if(BUILD_TEST)
#  enable_testing()
#  add_subdirectory(test)
#endif()

