SET(FPS_CORE_LIBRARY_SRCS 
  unscented.h
  depth_utils.h depth_utils.cpp
  cloud.h cloud.cpp
  base_camera_info.cpp base_camera_info.h
  multi_camera_info.cpp multi_camera_info.h
  camera_info_manager.cpp camera_info_manager.h
  base_projector.h base_projector.cpp
  base_correspondence_finder.cpp base_correspondence_finder.h
  solver.cpp solver.h
  base_aligner.cpp base_aligner.h
  pinhole_camera_info.cpp pinhole_camera_info.h
  pinhole_projector.cpp pinhole_projector.h
  projective_correspondence_finder.cpp projective_correspondence_finder.h
  projective_aligner.cpp projective_aligner.h
  multi_projector.h multi_projector.cpp
)

SET(FPS_CORE_LIBRARY_NN_SRCS 
  nn_correspondence_finder.cpp  nn_correspondence_finder.h
  nn_aligner.cpp  nn_aligner.h
)

if (FLANN_FOUND)
  SET(FPS_CORE_LIBRARY_SRCS ${FPS_CORE_LIBRARY_SRCS} ${FPS_CORE_LIBRARY_NN_SRCS})
  MESSAGE("ps_core: enabling nn support")
endif(FLANN_FOUND)

ADD_LIBRARY(fps_core_library SHARED
  ${FPS_CORE_LIBRARY_SRCS}
)


add_definitions(-DEIGEN_DONT_PARALLELIZE)	     
#add_definitions(-D_GO_PARALLEL_)	     

TARGET_LINK_LIBRARIES(fps_core_library 
  another_globals_library
  boss_library
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

if (FLANN_FOUND)
  TARGET_LINK_LIBRARIES(fps_core_library 
    ${FLANN_LIBRARIES} 
    )
else(FLANN_FOUND)
  MESSAGE("ps_core: nn support disabled")
endif(FLANN_FOUND)



