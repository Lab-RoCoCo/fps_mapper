ADD_LIBRARY(fps_loop_closer_library SHARED
  base_loop_closer.cpp base_loop_closer.h	
  trajectory_matcher_loop_closer.cpp trajectory_matcher_loop_closer.h	
  trajectory_matcher.cpp trajectory_matcher.h	
  trajectory_matcher_correspondence_finder.cpp trajectory_matcher_correspondence_finder.h	
  trajectory_matcher_solver.cpp trajectory_matcher_solver.h	
)

TARGET_LINK_LIBRARIES(fps_loop_closer_library 
  fps_map_library
)
