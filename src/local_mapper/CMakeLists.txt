ADD_LIBRARY(fps_local_mapper_library SHARED
  local_map_triggers.cpp  local_map_triggers.h
)

TARGET_LINK_LIBRARIES(fps_local_mapper_library 
  fps_core_library 
  fps_tracker_library
  fps_map_library
  boss_library
  gl_helpers_library
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_local_mapper_library 
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)

