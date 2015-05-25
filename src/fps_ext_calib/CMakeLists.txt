

# additional compiler flags, to produce more warnings
add_definitions(-std=c++11  -Wuninitialized  -Wall  -Wextra  -Wno-deprecated  -Wno-ignored-qualifiers  -Wno-unused-parameter)




add_executable( fps_ext_calib_node
  fps_ext_calib_node.cpp
  ext_calib_viewer.cpp
)

add_executable( fps_ext_calib_app
  fps_ext_calib_app.cpp
  ext_calib_viewer.cpp
)

TARGET_LINK_LIBRARIES(fps_ext_calib_node
  fps_ros_library
  ros_wrappers_library
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
  ${catkin_LIBRARIES}
)

TARGET_LINK_LIBRARIES(fps_ext_calib_app
  fps_ros_library
  ros_wrappers_library
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
  ${catkin_LIBRARIES}
)

if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_ext_calib_node
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

