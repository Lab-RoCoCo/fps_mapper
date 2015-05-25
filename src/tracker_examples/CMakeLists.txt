add_executable(fps_tracker_gui_app fps_tracker_gui_app.cpp)

TARGET_LINK_LIBRARIES(fps_tracker_gui_app
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


if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_tracker_gui_app
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)



#SGOTTTTTTTTT

add_executable(fps_tracker_gui_laido_app fps_tracker_gui_laido_app.cpp)

TARGET_LINK_LIBRARIES(fps_tracker_gui_laido_app
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


if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_tracker_gui_laido_app
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

