#pragma once

#include "qglviewer.h"
#include "qapplication.h"
#include "local_mapper_viewer.h"
#include "fps_map_viewers/trajectory_viewer.h"
#include "tracker/tracker.h"
#include "local_mapper/local_map_triggers.h"

namespace fps_mapper {

class LocalMapperViewer: public TrajectoryViewer{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  LocalMapperViewer(LocalMapTrigger* _trigger);
  virtual void draw();
protected:
  Tracker* _tracker;
  LocalMapTrigger* _trigger;
  bool _modelTainted;
  MapNodeList* _local_map_trajectory;
};


}
