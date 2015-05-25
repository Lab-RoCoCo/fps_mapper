#include "fps_mapper/RichPointMsg.h"
#include "fps_mapper/StampedCloudMsg.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "tf/transform_listener.h"
#include "fps_ros_msgs.h"

using namespace fps_mapper;
using namespace std;
  
txt_io::MessageWriter writer;
tf::TransformListener*  _listener = 0;

void cloudCallback(const StampedCloudMsgConstPtr& msg, Eigen::Isometry3f& pose, fps_mapper::Cloud& dest){
  if (_listener) {
    tf::StampedTransform transform;
    try{
      _listener->waitForTransform("tracker_origin_frame_id", 
				  msg->header.frame_id, 
				  msg->header.stamp, 
				  ros::Duration(0.1) );

      _listener->lookupTransform ("tracker_origin_frame_id", 
				  msg->header.frame_id, 
				  msg->header.stamp,  
				  transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    pose = tfTransform2eigen(transform);

    msg2cloud(dest,msg->cloud);
  }
}



int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  Eigen::Isometry3f ref_pose, curr_pose;
  fps_mapper::Cloud ref_cloud, curr_cloud;
  
  ros::Subscriber sub_curr = n.subscribe<fps_mapper::StampedCloudMsg>("/tracker/current_cloud", 10, boost::bind(cloudCallback, _1, curr_pose, curr_cloud));
  _listener = new tf::TransformListener(n);
  ros::spin();
}
