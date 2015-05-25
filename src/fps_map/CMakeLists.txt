ADD_LIBRARY(fps_map_library SHARED
  map_node.cpp map_node.h
  binary_node_relation.cpp binary_node_relation.h
  map_node_list.cpp map_node_list.h
  image_map_node.cpp image_map_node.h
  multi_image_map_node.cpp multi_image_map_node.h
  local_map.h local_map.cpp
)

TARGET_LINK_LIBRARIES(fps_map_library 
  fps_core_library 
  boss_library
  gl_helpers_library
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)


ADD_EXECUTABLE(fps_trajectory_loader_app fps_trajectory_loader_app.cpp)
TARGET_LINK_LIBRARIES(fps_trajectory_loader_app 
  fps_core_library 
  fps_map_library
  boss_library
  gl_helpers_library
  ${OpenCV_LIBS}
  ${ZLIB_LIBRARIES}   
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
  )

