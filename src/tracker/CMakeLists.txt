ADD_LIBRARY(fps_tracker_library SHARED
  tracker.cpp  tracker.h
  multi_tracker.cpp  multi_tracker.h
  base_triggers.cpp  base_triggers.h
  call_tracker_trigger.h call_tracker_trigger.cpp
)

TARGET_LINK_LIBRARIES(fps_tracker_library 
  another_globals_library
  fps_core_library 
  boss_library
  ${ZLIB_LIBRARIES} 
  ${OpenCV_LIBS}
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_tracker_library 
    ${FLANN_LIBRARIES}
    )
  ADD_DEFINITIONS(-D_FLANN_ENABLED_)
endif (FLANN_FOUND)
