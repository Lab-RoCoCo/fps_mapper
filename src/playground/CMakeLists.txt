# Find SuiteSparse
FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

#FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

if (FLANN_FOUND)
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
  add_executable(fps_refiner_app fps_refiner.cpp)
  TARGET_LINK_LIBRARIES(fps_refiner_app
    fps_global_optimization_library
    fps_map_viewers_library
    fps_tracker_library
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
    ${FLANN_LIBRARIES}
    ${G2O_OPENGL_HELPER_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_TYPES_SLAM3D} 
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${CSPARSE_LIBRARY}
    )
	
  add_executable(fps_loop_closer_app fps_loop_closer_app.cpp)
  TARGET_LINK_LIBRARIES(fps_loop_closer_app
    fps_loop_closer_library
    fps_global_optimization_library
    fps_map_viewers_library
    fps_tracker_library
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
    ${FLANN_LIBRARIES}
    ${G2O_OPENGL_HELPER_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_TYPES_SLAM3D} 
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${CSPARSE_LIBRARY}
  )

add_executable(fps_loop_closer fps_loop_closer.cpp)
  TARGET_LINK_LIBRARIES(fps_loop_closer
    fps_loop_closer_library
    fps_global_optimization_library
    fps_map_viewers_library
    fps_tracker_library
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
    ${FLANN_LIBRARIES}
    ${G2O_OPENGL_HELPER_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_TYPES_SLAM3D} 
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${CSPARSE_LIBRARY}
  )

endif (FLANN_FOUND)

add_executable(image_assembler image_assembler.cpp)
TARGET_LINK_LIBRARIES(image_assembler
  ${OpenCV_LIBS}
)


add_executable( bb_viewer
  bb_viewer.cpp
)
TARGET_LINK_LIBRARIES( bb_viewer
  fps_core_library 
  fps_map_library
  boss_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${ZLIB_LIBRARIES} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)






