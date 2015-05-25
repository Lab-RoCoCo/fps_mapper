#include "loop_closer_node_viewer.h"

#include "loop_closer/trajectory_matcher_loop_closer.h"

using namespace boss;
using namespace fps_mapper;
using namespace std;
using namespace GLHelpers;

namespace fps_mapper {  

  LoopCloserNodeViewer::LoopCloserNodeViewer(BaseLoopCloser* loop_closer_, 
				     boss::IdContext* context_): LocalMapListener(context_) {
    _need_redraw = true;
    _temp_nodes.clear();
    _loop_closer = loop_closer_;
    _current = 0;
    _previous = 0;
  }
  
  void LoopCloserNodeViewer::draw() {
    int attrs = ATTRIBUTE_SHOW;    
    attrs |= ATTRIBUTE_SELECTED;
    if(_current) { 
      if(!_previous) {
	  glColor4f(0, 0, 1, 1);
	  _current->draw(attrs);
      }
      for(BinaryNodeRelationSet::iterator it = relations.begin(); it != relations.end(); ++it) {
	BinaryNodeRelation* rel = it->get();
	if(rel->from() == _previous && rel->to() == _current) { 
	  glColor4f(0, 0, 1, 1);
	  _current->draw(attrs);
	  break;
	}
      }
    }
    for(BinaryNodeRelationSet::iterator it = _new_relations.begin(); it != _new_relations.end(); ++it) {      
      glColor4f(0, 1, 0, 0.33);
      it->get()->to()->draw(attrs);
    }    
    for(MapNodeList::iterator it = _candidate_closures.begin(); it != _candidate_closures.end(); ++it) {      
      glColor4f(1, 0, 0, 0.33);
      it->get()->draw(attrs);
    }    
    for(std::list<MapNode*>::iterator it = _temp_nodes.begin(); 
	it!=_temp_nodes.end(); 
	it++) {
      (*it)->draw();
    }

    TrajectoryViewer::draw();

    _need_redraw = false;
  }

  void LoopCloserNodeViewer::onNewLocalMap(LocalMap* lmap) {
    nodes.addElement(lmap);
    _previous = _current;
    _current = lmap;    
    _temp_nodes.clear();
    _need_redraw = true;
  }

  void LoopCloserNodeViewer::onNewNode(MapNode* n) {
    if(!n->parents().size()) { _temp_nodes.push_back(n); }
    _need_redraw = true;
  }

  void LoopCloserNodeViewer::onNewRelation(BinaryNodeRelation* r) {
    relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(r));
    r->to()->setTransform(r->from()->transform() * r->transform());

    if(r->from() == _previous && r->to() == _current) { 
      _bridge.psToG2o(relations, nodes);
      _bridge.quietOptimize(25);
      _bridge.g2oToPs(nodes);    
      updateGL();
      _new_relations.clear();      
      _candidate_closures.clear();
      _closures.clear();
      std::cerr << "[DEBUG]: auto " << _current << " transform is " << t2v(_current->transform()).transpose() << std::endl; 
      std::cerr << "[INFO]: processing local map " << _current << std::endl;
      std::cerr << "[INFO]: current map has " << nodes.size() << " nodes " << std::endl;
      std::cerr << "[INFO]: current map has " << relations.size() << " relations " << std::endl;
      _loop_closer->compute(_candidate_closures, _closures, _new_relations, _current, &_local_maps);    
      std::tr1::shared_ptr<MapNode> map_node = std::tr1::shared_ptr<MapNode>(_current);
      if(map_node) { _local_maps.push_back(map_node); }
      relations.insert(_new_relations.begin(), _new_relations.end());    
    }

    _need_redraw = true;
  }

  void LoopCloserNodeViewer::init(ros::NodeHandle& n) {
    LocalMapListener::init(n);
    setAxisIsDrawn(true);
    setBackgroundColor(QColor(128, 128, 128));
    _need_redraw = true;
  }

  void LoopCloserNodeViewer::postSelection(const QPoint&) {
    int id = selectedName();
    if(id < 0) { return; }
    MapNode* node = _names_map[id];
    LocalMap* lmap = dynamic_cast<LocalMap*>(node);
    if(_selected_objects.count(node)) { _selected_objects.erase(node); }
    else { _selected_objects.insert(node); }
  }

  void LoopCloserNodeViewer::keyPressEvent(QKeyEvent* e) {
    // Visualize all local maps
    if((e->key() == Qt::Key_V)) { 
      for(MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) { _selected_objects.insert(it->get()); }
      update();
    }
    // Clear viewer from local maps
    else if((e->key() == Qt::Key_C)) {
      _selected_objects.clear();
      _new_relations.clear();      
      _candidate_closures.clear();
      _closures.clear();
      _current = 0;
      update();
    }
    // Optimize the graph
    else if((e->key() == Qt::Key_O)) {
      _bridge.psToG2o(relations, nodes);
      _bridge.optimize();
      _bridge.g2oToPs(nodes);
      update();
    }
  }
  
}
