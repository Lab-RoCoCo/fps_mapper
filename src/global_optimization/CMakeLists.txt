# Find SuiteSparse
FIND_PACKAGE(SuiteSparse REQUIRED)
INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

set(CMAKE_MODULE_PATH "${G2O_ROOT}/cmake_modules;${CMAKE_MODULE_PATH}")
FIND_PACKAGE(G2O REQUIRED)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR})

INCLUDE_DIRECTORIES(${CSPARSE_INCLUDE_DIR})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --std=gnu++11")

add_library(fps_global_optimization_library
  g2o_bridge.cpp
)

TARGET_LINK_LIBRARIES(fps_global_optimization_library
    fps_core_library 
    fps_map_library
    boss_library
    gl_helpers_library
    ${ZLIB_LIBRARIES} 
    ${OpenCV_LIBS}
    ${Boost_FILESYSTEM_LIBRARY} 
    ${Boost_SYSTEM_LIBRARY}
    ${FLANN_LIBRARIES}
    ${G2O_OPENGL_HELPER_LIBRARY}
    ${G2O_SOLVER_CSPARSE_EXTENSION}
    ${G2O_TYPES_SLAM3D} 
    ${G2O_CORE_LIBRARY} 
    ${G2O_STUFF_LIBRARY} 
    ${CSPARSE_LIBRARY}
    )
