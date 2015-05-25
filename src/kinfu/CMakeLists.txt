find_package(PCL 1.8 REQUIRED)
find_package(OpenNI REQUIRED)
find_package(CUDA REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} "/usr/local/cuda-6.5/include")
link_directories(${PCL_LIBRARY_DIRS} )
add_definitions(${PCL_DEFINITIONS})

cuda_add_executable(fps_kinfu_tracker_gui_app fps_kinfu_tracker_gui_app.cpp
  fps_kinfu_tracker.cpp fps_kinfu_tracker.h
  kinfu_tracker_viewer.cpp kinfu_tracker_viewer.h
)
target_link_libraries(fps_kinfu_tracker_gui_app 
  ${PCL_LIBRARIES} 
  fps_tracker_viewers_library
  fps_tracker_library
  txt_io_library
  fps_core_library 
  another_globals_library
  boss_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)