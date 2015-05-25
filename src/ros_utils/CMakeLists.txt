add_executable(odom_tf_publisher_node  odom_tf_publisher_node.cpp)
TARGET_LINK_LIBRARIES(odom_tf_publisher_node
  ros_wrappers_library
  txt_io_library
  gl_helpers_library
  ${catkin_LIBRARIES}
  )


add_executable(fps_message_dumper_node 
  fps_message_dumper_node.cpp 
  )
TARGET_LINK_LIBRARIES(fps_message_dumper_node 
  ros_wrappers_library
  txt_io_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${catkin_LIBRARIES}
  )


