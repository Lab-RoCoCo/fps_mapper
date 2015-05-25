#pragma once
#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "tf/transform_listener.h"
#include "gl_helpers/simple_viewer.h"
#include <qapplication.h>
#include "fps_map_viewers/trajectory_viewer.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "local_map_listener.h"
#include "tf/transform_listener.h"
#include <qevent.h>

namespace fps_mapper {  

  class LocalMapViewer: public LocalMapListener, public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    LocalMapViewer(boss::IdContext* context_=0);

    virtual void draw();
    virtual void onNewLocalMap(LocalMap* lmap);
    virtual void onNewNode(MapNode * n);
    virtual void onNewRelation(BinaryNodeRelation * r);
    void cloudCallback(const StampedCloudMsgConstPtr& msg, Eigen::Isometry3f* pose, fps_mapper::Cloud* dest);

    inline void setOriginFrameId(const std::string frame_id) {_origin_frame_id = frame_id;}
    inline const std::string& originFrameId() const {return _origin_frame_id;}
    
    void keyPressEvent(QKeyEvent *e);

    void setShowCurrentClouds(bool f);
    inline bool showCurrentClouds() const { return _show_clouds; }
    inline bool needRedraw() const {return _need_redraw;}
    void init(ros::NodeHandle& n, tf::TransformListener* tf_listener);

  protected:
    Cloud _reference;
    Cloud _current;
    bool _need_redraw;
    ros::NodeHandle * _n;
    std::string _origin_frame_id;
    ros::Subscriber _curr_sub, _ref_sub;
    Eigen::Isometry3f _curr_pose, _ref_pose;
    fps_mapper::Cloud _curr_cloud, _ref_cloud;
    tf::TransformListener* _tf_listener;
    std::list<MapNode*> _temp_nodes; // nodes not yet in any of the local maps
    bool _show_clouds;
  };

}
