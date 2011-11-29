find_package(GTest)

if(NOT GTEST_FOUND)
  message(STATUS "Disabling testing since gtest was not found.")
  return()
endif()

include_directories(${GTEST_INCLUDE_DIRS})

add_executable(capture-test
  main.cpp
  )

target_link_libraries(capture-test
  ${OpenCV_LIBS}
  ${GTEST_LIBRARIES}
  )

add_test(capture-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/capture-test)
