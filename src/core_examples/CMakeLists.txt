
add_executable(fps_projective_aligner_test fps_projective_aligner_test.cpp)
TARGET_LINK_LIBRARIES(fps_projective_aligner_test 
  fps_core_library 
  boss_library
  ${OpenCV_LIBS} 
  ${OPENGL_glu_LIBRARY}
  ${OPENGL_gl_LIBRARY} 
  ${Boost_FILESYSTEM_LIBRARY} 
  ${Boost_SYSTEM_LIBRARY}
)

if (FLANN_FOUND)
  add_executable(fps_nn_aligner_test fps_nn_aligner_test.cpp)
  TARGET_LINK_LIBRARIES(fps_nn_aligner_test 
    fps_core_library 
    boss_library
    ${OpenCV_LIBS} 
    ${FLANN_LIBRARIES} 
    ${OPENGL_glu_LIBRARY}
    ${OPENGL_gl_LIBRARY} 
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY}
    )
endif(FLANN_FOUND)
