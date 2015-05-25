#include <iostream>
#include <cstring>
#include "globals/system_utils.h"

#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include "cloud_publisher_trigger.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "tf/transform_listener.h"
#include "gl_helpers/simple_viewer.h"
#include <qapplication.h>
#include "fps_map_viewers/trajectory_viewer.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "local_map_listener.h"
#include "local_map_viewer.h"

using namespace boss;
using namespace fps_mapper;
using namespace GLHelpers;
using namespace std;

const char* banner[]= {
  "fps_local_mapper_viewer_node: shows the local maps under construction",
  "usage:",
  " fps_local_mapper_dumper_node ",
  "once the gus starts",
  "1: toggles/untoggles the current view (and saves a lot of bandwidth)",
  "shift-left-click on a node: highlights the local map of the node",
  0
};
  

int main(int argc, char** argv) {
  if (argc>1 && ! strcmp(argv[1],"-h")){
    system_utils::printBanner(banner);
    return 0;
  }

  ros::init(argc, argv, "local_map_viewer");
  ros::NodeHandle n;

  tf::TransformListener* listener = new tf::TransformListener(ros::Duration(60.0));

  QApplication* app=0; 
  LocalMapViewer* viewer=0;
  
  app=new QApplication(argc, argv);
  viewer = new LocalMapViewer();
  viewer->show();
  viewer->init(n, listener);
    
  while(ros::ok()){
    ros::spinOnce();
    app->processEvents();
    if (viewer->needRedraw()) {			
      viewer->updateGL();
    }
  }

}
