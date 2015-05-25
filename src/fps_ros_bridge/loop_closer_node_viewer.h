#pragma once

#include <ros/ros.h>
#include <qapplication.h>
#include <qevent.h>

#include "fps_mapper/StampedCloudMsg.h"
#include "fps_map_viewers/trajectory_viewer.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "global_optimization/g2o_bridge.h"
#include "loop_closer/base_loop_closer.h"
#include "tf/transform_listener.h"
#include "txt_io/message_writer.h"

#include "fps_ros_bridge/fps_ros_msgs.h"
#include "fps_ros_bridge/local_map_listener.h"

namespace fps_mapper {  

  class LoopCloserNodeViewer: public LocalMapListener, public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    LoopCloserNodeViewer(BaseLoopCloser* loop_closer_, boss::IdContext* context_ = 0);

    virtual void draw();
    virtual void onNewLocalMap(LocalMap* lmap);
    virtual void onNewNode(MapNode* n);
    virtual void onNewRelation(BinaryNodeRelation* r);
    virtual void onNewCameraInfo(BaseCameraInfo* cam) {}
    virtual void postSelection(const QPoint& point);

    inline bool needRedraw() const { return _need_redraw; }
    
    void init(ros::NodeHandle& n);

    void keyPressEvent(QKeyEvent* e);

  protected:
    bool _need_redraw;
    LocalMap* _current;
    LocalMap* _previous;
    std::list<MapNode*> _temp_nodes;
    MapNodeList _candidate_closures;
    MapNodeList _closures;
    BinaryNodeRelationSet _new_relations;
    MapNodeList _local_maps;
    BaseLoopCloser* _loop_closer;
    G2OBridge _bridge;
  };

}
