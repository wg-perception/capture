#testing macros
macro(pytest pyfile)
  string(REPLACE ";" " " args "${ARGN}")
  add_test(object_recognition_${pyfile}
    ${CMAKE_CURRENT_SOURCE_DIR}/run_test.sh ${CMAKE_BINARY_DIR}/python_path.sh ${CMAKE_CURRENT_SOURCE_DIR}/${pyf$
    )
endmacro()

object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/capture)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/orb_template.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/orb_track.py)
object_recognition_core_help_test(${CMAKE_CURRENT_SOURCE_DIR}/../apps/upload)
