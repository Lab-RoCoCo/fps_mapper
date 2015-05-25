add_library(ros_wrappers_library 
  image_message_listener.cpp 
  imu_interpolator.cpp
  odom_tf_publisher.cpp
  ros_utils.cpp
)

TARGET_LINK_LIBRARIES(ros_wrappers_library
  txt_io_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${FLANN_LIBRARIES}   
  ${catkin_LIBRARIES}
  )



