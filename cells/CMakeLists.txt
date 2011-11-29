include_directories(${CMAKE_CURRENT_SOURCE_DIR})
#deal with the objcog module
ectomodule(capture
  module.cpp
  DeltaRT.cpp
  RescaleRegisteredDepth.cpp
  )

link_ecto(capture
  object_recognition_db
  opencv_core
  )

install_ecto_module(capture)

