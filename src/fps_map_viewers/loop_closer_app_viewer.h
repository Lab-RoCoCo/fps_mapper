#pragma once

#include <qapplication.h>
#include <qevent.h>

#include "trajectory_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "global_optimization/g2o_bridge.h"
#include "loop_closer/base_loop_closer.h"

namespace fps_mapper {  

  class LoopCloserAppViewer: public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    LoopCloserAppViewer(BaseLoopCloser* loop_closer_);
    
    void init();

    virtual void draw();    
    virtual void postSelection(const QPoint& point);

    void keyPressEvent(QKeyEvent* e);

  protected:
    std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
    LocalMap *_reference, *_current;
    BaseLoopCloser* _loop_closer;
    G2OBridge _bridge;
    MapNodeList _candidate_closures;
    MapNodeList _closures;
    BinaryNodeRelationSet _new_relations;      
  };

}
