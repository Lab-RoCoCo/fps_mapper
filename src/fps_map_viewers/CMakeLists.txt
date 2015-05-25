ADD_LIBRARY(fps_map_viewers_library SHARED
  trajectory_viewer.cpp trajectory_viewer.h
  loop_closer_app_viewer.cpp loop_closer_app_viewer.h
  loop_closer_viewer.cpp loop_closer_viewer.h
)

TARGET_LINK_LIBRARIES( fps_map_viewers_library
  fps_global_optimization_library
  fps_loop_closer_library
  fps_map_library 
  fps_core_library 
  fps_map_library
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


ADD_EXECUTABLE(fps_trajectory_viewer_app fps_trajectory_viewer_app.cpp)
TARGET_LINK_LIBRARIES(fps_trajectory_viewer_app 
  fps_map_viewers_library
  fps_core_library 
  fps_map_library
  boss_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  )
