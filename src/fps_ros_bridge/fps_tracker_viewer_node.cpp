#include "tracker/tracker.h"
#include "fps_mapper/RichPointMsg.h"
#include "fps_mapper/StampedCloudMsg.h"
#include "cloud_publisher_trigger.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "tf/transform_listener.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"

#include "qapplication.h"
#include "txt_io/message_reader.h"
#include "fps_ros_msgs.h"

using namespace fps_mapper;
using namespace fps_mapper;
using namespace GLHelpers;
using namespace std;

class TrackerViewerNode: public GLHelpers::SimpleViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  void cloudCallback(const StampedCloudMsgConstPtr& msg, Eigen::Isometry3f* pose, fps_mapper::Cloud* dest){
    cerr << "got cloud: " << endl;
    tf::StampedTransform transform;
    try{
      _tf_listener->waitForTransform("/tracker_origin_frame_id", 
				  msg->header.frame_id, 
				  msg->header.stamp, 
				  ros::Duration(0.1) );

      _tf_listener->lookupTransform ("/tracker_origin_frame_id", 
				  msg->header.frame_id, 
				  msg->header.stamp,  
				  transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    *pose = tfTransform2eigen(transform);

    msg2cloud(*dest,msg->cloud);
    _need_redraw = true;

    cerr << "receive: Draw: curr: " << _curr_cloud.size() << " ref: " << _ref_cloud.size() << endl;
  }

  TrackerViewerNode (ros::NodeHandle& nh, tf::TransformListener* tf_listener) {
    _tf_listener = tf_listener;
    _curr_sub = nh.subscribe<fps_mapper::StampedCloudMsg>("/tracker/current_cloud", 10, boost::bind(&TrackerViewerNode::cloudCallback, this, _1, &_curr_pose, &_curr_cloud));

    _ref_sub = nh.subscribe<fps_mapper::StampedCloudMsg>("/tracker/reference_cloud", 10, boost::bind(&TrackerViewerNode::cloudCallback, this, _1, &_ref_pose, &_ref_cloud));
    _curr_pose.setIdentity();
    _ref_pose.setIdentity();
    _need_redraw = true;
  }

  

  inline bool needRedraw() const {return _need_redraw; }

  virtual void draw() {
    glPushMatrix();
    glMultMatrix(_ref_pose);
    glColor3f(0.5, 0.5, 0.5);
    _ref_cloud.draw();
    glPopMatrix();

    glPushMatrix();
    glMultMatrix(_curr_pose);

    glPushMatrix();
    glColor3f(0.2, 0.2, 1.0);
    _curr_cloud.draw();
    glScalef(0.2, 0.2, 0.2);
    GLHelpers::drawReferenceSystem();
    glPopMatrix();

    glPopMatrix();

    //cerr << "Draw: curr: " << _curr_cloud.size() << " ref: " << _ref_cloud.size() << endl;
  }

protected:

  tf::TransformListener*  _tf_listener;
  bool _need_redraw;
  ros::Subscriber _curr_sub, _ref_sub;
  Eigen::Isometry3f _curr_pose, _ref_pose;
  fps_mapper::Cloud _curr_cloud, _ref_cloud;

};


int main(int argc, char** argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  tf::TransformListener listener(ros::Duration(60.0));
  
  QApplication* app=new QApplication(argc, argv);
  TrackerViewerNode* viewer = new TrackerViewerNode(n, &listener);
  viewer->show();

  while(ros::ok()) {
    if (viewer->needRedraw())
      viewer->updateGL();
    app->processEvents();
    ros::spinOnce();
  }
}
