
add_library(fps_ros_library
  local_map_listener.cpp
  cloud_publisher_trigger.cpp
  fps_ros_msgs.cpp
  local_mapper_ros.cpp
)

TARGET_LINK_LIBRARIES( fps_ros_library
  fps_map_library
  fps_local_mapper_library
  fps_tracker_library
  fps_core_library 
  boss_library
  ros_wrappers_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
)


add_library(fps_ros_viewers_library
  local_map_viewer.cpp
  global_map_viewer.cpp
  loop_closer_node_viewer.cpp
)

TARGET_LINK_LIBRARIES( fps_ros_viewers_library
  fps_ros_library
  fps_map_library
  fps_map_viewers_library
  fps_loop_closer_library
  fps_global_optimization_library
  fps_core_library 
  boss_library
  txt_io_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
)


add_executable(fps_tracker_node  
  fps_tracker_node.cpp 
  cloud_publisher_trigger.cpp
)

TARGET_LINK_LIBRARIES(fps_tracker_node 
  fps_ros_library
  fps_tracker_viewers_library
  fps_tracker_library
  txt_io_library
  fps_core_library 
  boss_library
  ros_wrappers_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


add_executable(fps_tracker_viewer_node  
  fps_tracker_viewer_node.cpp
)

TARGET_LINK_LIBRARIES(fps_tracker_viewer_node 
  fps_ros_library
  fps_map_library 
  fps_tracker_library
  ros_wrappers_library
  txt_io_library
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
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


add_executable(fps_local_mapper_node  
  fps_local_mapper_node.cpp
)

TARGET_LINK_LIBRARIES(fps_local_mapper_node 
  fps_ros_library
  fps_local_mapper_library
  fps_tracker_library
  ros_wrappers_library
  txt_io_library
  fps_core_library 
  boss_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )



add_executable(fps_global_mapper_viewer_node  
  fps_global_mapper_viewer_node.cpp
)

TARGET_LINK_LIBRARIES(fps_global_mapper_viewer_node 
  fps_ros_viewers_library
  fps_ros_library
  fps_map_library
  fps_map_viewers_library
  #fps_local_mapper_viewers_library
  #fps_local_mapper_library
  #fps_tracker_library
  #txt_io_library
  fps_core_library 
  boss_library
  txt_io_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )



add_executable(fps_local_mapper_viewer_node  
  fps_local_mapper_viewer_node.cpp
)

TARGET_LINK_LIBRARIES(fps_local_mapper_viewer_node 
  fps_ros_viewers_library
  fps_ros_library
  fps_map_library
  fps_map_viewers_library
  #fps_local_mapper_viewers_library
  #fps_local_mapper_library
  #fps_tracker_library
  #txt_io_library
  fps_core_library 
  boss_library
  txt_io_library
  gl_helpers_library
  ${QGLVIEWER_LIBRARY} 
  ${QT_QTXML_LIBRARY} 
  ${QT_QTOPENGL_LIBRARY} 
  ${QT_QTGUI_LIBRARY} 
  ${QT_QTCORE_LIBRARY} 
  ${OPENGL_gl_LIBRARY} 
  ${OPENGL_glu_LIBRARY}
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )



add_executable(fps_local_mapper_client_node  
  fps_local_mapper_client_node.cpp
)

TARGET_LINK_LIBRARIES(fps_local_mapper_client_node 
  fps_ros_library
  fps_map_library
  fps_map_viewers_library
  fps_core_library 
  boss_library
  gl_helpers_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


add_executable(fps_local_mapper_dumper_node  
  fps_local_mapper_dumper_node.cpp
)

TARGET_LINK_LIBRARIES(fps_local_mapper_dumper_node 
  fps_ros_library
  fps_map_library
  fps_map_viewers_library
  fps_core_library 
  boss_library
  gl_helpers_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


# Find SuiteSparse
FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

set(CMAKE_MODULE_PATH "${G2O_ROOT}/cmake_modules;${CMAKE_MODULE_PATH}")
FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

if (FLANN_FOUND)
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
  add_executable(fps_refiner_node 
    fps_refiner_node.cpp
    )
  TARGET_LINK_LIBRARIES(fps_refiner_node
    fps_ros_viewers_library
    fps_ros_library
    fps_global_optimization_library
    fps_map_library
    fps_map_viewers_library
    fps_core_library 
    boss_library
    txt_io_library
    gl_helpers_library
    ${QGLVIEWER_LIBRARY} 
    ${QT_QTXML_LIBRARY} 
    ${QT_QTOPENGL_LIBRARY} 
    ${QT_QTGUI_LIBRARY} 
    ${QT_QTCORE_LIBRARY} 
    ${OPENGL_gl_LIBRARY} 
    ${OPENGL_glu_LIBRARY}
    ${OpenCV_LIBS}
    ${ZLIB_LIBRARIES}   
    ${FLANN_LIBRARIES}   
    ${catkin_LIBRARIES}
    ${G2O_OPENGL_HELPER_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_TYPES_SLAM3D} 
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${CSPARSE_LIBRARY}
    )
endif (FLANN_FOUND)








# add_executable(ros_utils_test 
#   ros_utils_test.cpp  
#   ros_msgs.cpp
# )


# TARGET_LINK_LIBRARIES(ros_utils_test 
#   fps_ros_library
#   txt_io_library
#   fps_core_library 
#   boss_library
#   gl_helpers_library
#   ${OPENGL_gl_LIBRARY} 
#   ${OPENGL_glu_LIBRARY}
#   ${OpenCV_LIBS}
#   ${ZLIB_LIBRARIES}   
#   ${FLANN_LIBRARIES}   
#   ${catkin_LIBRARIES}
#   fps_ros_library
#   fps_map_library 
#   fps_tracker_library
#   txt_io_library
#   fps_core_library 
#   boss_library
#   gl_helpers_library
#   ${QGLVIEWER_LIBRARY} 
#   ${QT_QTXML_LIBRARY} 
#   ${QT_QTOPENGL_LIBRARY} 
#   ${QT_QTGUI_LIBRARY} 
#   ${QT_QTCORE_LIBRARY} 
#   ${OPENGL_gl_LIBRARY} 
#   ${OPENGL_glu_LIBRARY}
#   ${OpenCV_LIBS}
#   ${ZLIB_LIBRARIES}   
#   ${FLANN_LIBRARIES}   
#   ${catkin_LIBRARIES}
#   )





# add_executable(fps_tracker_client_node  
#   fps_tracker_client_node.cpp
# )

# TARGET_LINK_LIBRARIES(fps_tracker_client_node 
#   fps_ros_library
#   fps_map_library 
#   fps_tracker_library
#   ros_wrappers_library
#   txt_io_library
#   fps_core_library 
#   boss_library
#   gl_helpers_library
#   ${OPENGL_gl_LIBRARY} 
#   ${OPENGL_glu_LIBRARY}
#   ${OpenCV_LIBS}
#   ${ZLIB_LIBRARIES}   
#   ${FLANN_LIBRARIES}   
#   ${catkin_LIBRARIES}
#   )
