#include "globals/system_utils.h"
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "txt_io/message_dumper_trigger.h"
#include "ros_wrappers/imu_interpolator.h"
#include "ros_wrappers/image_message_listener.h"

using namespace std;
using namespace fps_mapper;
using namespace txt_io;
using namespace Eigen;
using namespace system_utils;

const char* banner[]= {
  "fps_message_dumper_node",
  "simple file dumper for depth tracker",
  "usage:",
  " start the kinect/xtion node or play a bag that contains depth images",
  " in a shell type",
  " rosrun fps_mapper fps_message_dumper_node [options]",
  " where [options]: ",
  "  -t:  [string] ros topic, default: /camera/depth/image_raw",
  "  -base_link_frame_id <base frame id> (default unset, if provided it will consider the odometry)",
  "  -o:       [string] output filename where to write the local maps. Default: out.txt",
  0
};


int main(int argc, char **argv) {
  std::string outputFilename="out.txt";
  std::string outputFileDir;
  int c = 1;
  std::list<string> topics;
  std::list<ImageMessageListener*> camera_listeners;
  std::string imu_topic = "";
  tf::TransformListener * listener = 0;
 std::string base_link_frame_id = "";
 std::string odom_frame_id = "/odom"; 

  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      printBanner(banner);
      return 0;
    }
    if (! strcmp(argv[c], "-t")){
      c++;
      topics.push_back(argv[c]);
    }
    if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id=argv[c];
    }
    if (! strcmp(argv[c], "-imu")){
      c++;
      imu_topic=argv[c];
    }
    if (! strcmp(argv[c], "-o")){
      c++;
      outputFilename = argv[c];
    }
    c++;
  }
  
  ros::init(argc, argv, "fps_message_dumper_node");
  if (base_link_frame_id.length()>0){
    cerr << "making listener" << endl;
    listener = new tf::TransformListener();
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);

  ImuInterpolator* interpolator = 0;
  if (imu_topic.length()){
    interpolator = new ImuInterpolator(&nh);
    interpolator->subscribe(imu_topic);
  }

  SensorMessageSorter sorter;
  MessageWriter writer;
  writer.open(outputFilename);
  MessageDumperTrigger* dumper = new MessageDumperTrigger(&sorter, 0, &writer, outputFileDir+"/");
  for (std::list<std::string>::iterator it = topics.begin(); it!=topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
    if (interpolator)
      camera_listener->setImuInterpolator(interpolator);
    camera_listener->subscribe(topic);
    camera_listener->setVerbose(true);
    cerr << "subscribing for topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }
  ros::spin();
}



