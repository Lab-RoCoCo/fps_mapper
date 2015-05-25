ADD_LIBRARY(fps_local_mapper_viewers_library SHARED
  local_mapper_viewer.cpp local_mapper_viewer.h
)

TARGET_LINK_LIBRARIES(fps_local_mapper_viewers_library 
  fps_local_mapper_library
  fps_map_viewers_library
  fps_core_library 
  fps_tracker_library
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

if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_local_mapper_viewers_library 
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

