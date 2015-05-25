#pragma once

#include <qapplication.h>
#include <qevent.h>

#include "boss/serializer.h"

#include "trajectory_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "global_optimization/g2o_bridge.h"
#include "loop_closer/base_loop_closer.h"

namespace fps_mapper {  

  class LoopCloserViewer: public TrajectoryViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    LoopCloserViewer(BaseLoopCloser* loop_closer_, 
		     std::string output_filename_ = "", std::list<boss::Serializable*>* serializable_objects_ = 0);

    bool startToFeed() const { return _start_to_feed; }
    bool needRedraw() const { return _need_redraw; }
    MapNode* currentLocalMap() const { return _current_local_map; }

    void setNeedRedraw(const bool need_redraw_) { _need_redraw = need_redraw_; }
    void setStartToFeed(const bool start_to_feed_) { _start_to_feed = start_to_feed_; }
    void setCurrentLocalMap(MapNode* current_local_map_) {
      _previous_local_map = current_local_map_;
      _current_local_map = current_local_map_; 
      findLoopClosures(_current_local_map);
    }
    
    void init();
    void findLoopClosures(MapNode* map_node);

    virtual void draw();    
    virtual void keyPressEvent(QKeyEvent* e);
    virtual void postSelection(const QPoint& point);

  protected:
    int _counter;
    bool _need_redraw;
    bool _start_to_feed;
    int _graph_optimization_itearations;
    std::string _output_filename;
    std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
    MapNode* _current_local_map;
    MapNode* _previous_local_map;
    MapNode* _selected_local_map;
    MapNode* _candidate_local_map;
    BaseLoopCloser* _loop_closer;
    G2OBridge _bridge;
    std::list<boss::Serializable*>* _serializable_objects;
    MapNodeList _candidate_closures;
    MapNodeList _closures;
    BinaryNodeRelationSet _new_relations;      
  };

}
