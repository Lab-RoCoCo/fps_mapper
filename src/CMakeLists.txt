#LIST( APPEND CMAKE_MODULE_PATH "/home/johann/repo-bonn/g2o/cmake_modules" )
#FIND_PACKAGE(G2O REQUIRED)


# Include the subdirectories
ADD_SUBDIRECTORY(globals)
ADD_SUBDIRECTORY(boss)
ADD_SUBDIRECTORY(gl_helpers)
ADD_SUBDIRECTORY(txt_io)
ADD_SUBDIRECTORY(core)
ADD_SUBDIRECTORY(core_examples)
ADD_SUBDIRECTORY(core_viewers)
ADD_SUBDIRECTORY(tracker)
ADD_SUBDIRECTORY(tracker_viewers)
ADD_SUBDIRECTORY(tracker_examples)
ADD_SUBDIRECTORY(fps_map)
ADD_SUBDIRECTORY(fps_map_viewers)
ADD_SUBDIRECTORY(local_mapper)
ADD_SUBDIRECTORY(local_mapper_viewers)
ADD_SUBDIRECTORY(local_mapper_examples)
ADD_SUBDIRECTORY(calibration)
ADD_SUBDIRECTORY(ros_wrappers)
ADD_SUBDIRECTORY(ros_utils)
ADD_SUBDIRECTORY(fps_ext_calib)

ADD_SUBDIRECTORY(fps_ros_bridge)
#ADD_SUBDIRECTORY(traversability)
#ADD_SUBDIRECTORY(traversability_ros_nodes)

# the one below requires g2o
ADD_SUBDIRECTORY(global_optimization)
ADD_SUBDIRECTORY(playground)

