#include "local_map_viewer.h"
#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "gl_helpers/simple_viewer.h"
#include "fps_map_viewers/trajectory_viewer.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "local_map_listener.h"

using namespace boss;
using namespace fps_mapper;
using namespace std;
using namespace GLHelpers;

namespace fps_mapper {  

  LocalMapViewer::LocalMapViewer(boss::IdContext* context_):
    LocalMapListener(context_){
    _origin_frame_id= "tracker_origin_frame_id";
    _show_clouds = true;
  }
  
  void LocalMapViewer::draw() {

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
    for (std::list<MapNode*>::iterator it = _temp_nodes.begin(); it!=_temp_nodes.end(); it++){
      (*it)->draw();
    }
    TrajectoryViewer::draw();
    _need_redraw = false;
  }


  void LocalMapViewer::onNewLocalMap(LocalMap* lmap) {
    _temp_nodes.clear();
    cerr << "Got new local map!!!!!!"<< endl;
    _need_redraw = true;
    nodes.addElement(lmap);
  }

  void LocalMapViewer::onNewNode(MapNode * n) {
    if (!n->parents().size())
      _temp_nodes.push_back(n);
    _need_redraw = true;
  }

  void LocalMapViewer::onNewRelation(BinaryNodeRelation * r) {
    relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(r));
    _need_redraw = true;
  }

  void LocalMapViewer::cloudCallback(const StampedCloudMsgConstPtr& msg, Eigen::Isometry3f* pose, Cloud* dest){
    tf::StampedTransform transform;
    try{
      _tf_listener->waitForTransform(_origin_frame_id, 
				     msg->header.frame_id, 
				     msg->header.stamp, 
				     ros::Duration(0.1) );

      _tf_listener->lookupTransform (_origin_frame_id, 
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

  void LocalMapViewer::setShowCurrentClouds(bool show_clouds) {
    if (show_clouds == _show_clouds)
      return;
    _show_clouds = show_clouds;
    if (_show_clouds) {
      _curr_sub = _n->subscribe<fps_mapper::StampedCloudMsg>("/tracker/current_cloud", 10, boost::bind(&LocalMapViewer::cloudCallback, this, _1, &_curr_pose, &_curr_cloud));

      _ref_sub = _n->subscribe<fps_mapper::StampedCloudMsg>("/tracker/reference_cloud", 10, boost::bind(&LocalMapViewer::cloudCallback, this, _1, &_ref_pose, &_ref_cloud));
      _curr_pose.setIdentity();
      _ref_pose.setIdentity();
    } else {
      _curr_sub.shutdown();
      _ref_sub.shutdown();
    }
  }

  void LocalMapViewer::keyPressEvent(QKeyEvent *e){
    if ((e->key() == Qt::Key_1)){
      setShowCurrentClouds(! showCurrentClouds());
    }
  }

  void LocalMapViewer::init(ros::NodeHandle& n, tf::TransformListener* tf_listener){
    LocalMapListener::init(n);
    _n = &n;
    _tf_listener = tf_listener;
    _show_clouds = false;
    setShowCurrentClouds(true);
    _need_redraw = true;
  }
  
}
