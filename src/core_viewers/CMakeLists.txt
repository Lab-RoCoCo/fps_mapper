ADD_LIBRARY(fps_core_viewers_library SHARED
  cloud_viewer.cpp cloud_viewer.h
)

TARGET_LINK_LIBRARIES(fps_core_viewers_library
  another_globals_library
  fps_core_library 
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
  TARGET_LINK_LIBRARIES(fps_core_viewers_library 
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

add_executable(fps_cloud_viewer_app fps_cloud_viewer_app.cpp)

TARGET_LINK_LIBRARIES(fps_cloud_viewer_app
  another_globals_library
  fps_core_viewers_library
  fps_core_library 
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
  TARGET_LINK_LIBRARIES(fps_cloud_viewer_app
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

add_executable(fps_aligner_gui_app fps_aligner_gui_app.cpp)

TARGET_LINK_LIBRARIES(fps_aligner_gui_app
  another_globals_library
  fps_core_viewers_library
  fps_core_library 
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
  TARGET_LINK_LIBRARIES(fps_aligner_gui_app
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)
