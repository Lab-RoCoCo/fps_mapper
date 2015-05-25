#pragma once
#include "local_map_viewer.h"
#include "fps_mapper/MapUpdateMsg.h"

namespace fps_mapper {
  class GlobalMapViewer: public LocalMapViewer{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GlobalMapViewer(boss::IdContext* context = 0);
    void updateCallback(const MapUpdateMsgConstPtr& msg);
    void init(ros::NodeHandle& n, tf::TransformListener* tf_listener);
    virtual void draw();
    virtual void onNewNode(MapNode* node);

  protected:
    ros::Subscriber _updates_sub;
    Eigen::Isometry3f _delta_pose; // transform between the global map and the local mapper one
    // required to draw correclty the unassigned trajectory chunks
  };
}
