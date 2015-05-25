add_library(fps_extrinsic_calibrator_library
  calibration_measurement.h calibration_measurement.cpp
  calibration_prior.h calibration_prior.cpp
  camera_calibration_solver.h camera_calibration_solver.cpp
  extrinsic_calibrator_trigger.h extrinsic_calibrator_trigger.cpp)

TARGET_LINK_LIBRARIES(fps_extrinsic_calibrator_library
  fps_tracker_viewers_library
  fps_tracker_library
  fps_core_library 
  fps_map_library 
  txt_io_library
  boss_library
  gl_helpers_library
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
  #${G2O_TYPES_SLAM3D_ADDONS} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  )


add_executable(fps_extrinsic_calibrator_gui_app 
  fps_extrinsic_calibrator_gui_app.cpp)

TARGET_LINK_LIBRARIES(fps_extrinsic_calibrator_gui_app
  fps_extrinsic_calibrator_library
  fps_tracker_viewers_library
  fps_tracker_library
  fps_core_library 
  fps_map_library 
  txt_io_library
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
  #${G2O_TYPES_SLAM3D_ADDONS} 
  ${G2O_CORE_LIBRARY} 
  ${G2O_STUFF_LIBRARY} 
  ${CSPARSE_LIBRARY}
  )


add_executable(fps_plane_fit_app fps_plane_fit_app.cpp)
TARGET_LINK_LIBRARIES(fps_plane_fit_app 
  fps_core_library 
  txt_io_library
  fps_map_library
  boss_library
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  ${FLANN_LIBRARIES}
  )
