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

#testing macros
macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(object_recognition_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyf$
    )
endmacro()

object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/capture)
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/orb_template.py)
object_recognition_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/orb_track.py)


