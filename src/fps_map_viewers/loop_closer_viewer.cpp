#include "loop_closer_viewer.h"

#include "loop_closer/trajectory_matcher_loop_closer.h"

using namespace boss;
using namespace fps_mapper;
using namespace std;
using namespace GLHelpers;

namespace fps_mapper {  
  
  LoopCloserViewer::LoopCloserViewer(BaseLoopCloser* loop_closer_, 
				     std::string output_filename_, 
				     std::list<Serializable*>* serializable_objects_): TrajectoryViewer() { 
    _counter = 0;
    _loop_closer = loop_closer_;     
    _need_redraw = true;
    _start_to_feed = false;
    _graph_optimization_itearations = 25;
    _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
    _current_local_map = 0;
    _previous_local_map = 0;
    _selected_local_map = 0;
    _candidate_local_map = 0;
    _output_filename = output_filename_; 
    _serializable_objects = serializable_objects_;
  }
  
  void LoopCloserViewer::draw() {
    int attrs = ATTRIBUTE_SHOW;    
    attrs |= ATTRIBUTE_SELECTED;

    if(_current_local_map) { 
      glColor4f(0, 0, 1, 1);
      _current_local_map->draw(attrs);
    }
    for(BinaryNodeRelationSet::iterator it = _new_relations.begin(); it != _new_relations.end(); ++it) {      
      glColor4f(0, 1, 0, 0.33);
      it->get()->to()->draw(attrs);
    }    
    for(MapNodeList::iterator it = _candidate_closures.begin(); it != _candidate_closures.end(); ++it) {      
      glColor4f(1, 0, 0, 0.33);
      it->get()->draw(attrs);
    }    

    TrajectoryViewer::draw();

    // char buffer[2048];
    // sprintf(buffer, "snapshot_%05d.png", _counter);
    // _counter++;
    // this->setSnapshotFormat(QString("PNG"));
    // this->setSnapshotQuality(10);
    // this->saveSnapshot(QString(buffer), true);

    _need_redraw = false;
  }

  void LoopCloserViewer::init() {
    SimpleViewer::init();
    setAxisIsDrawn(true);
    setBackgroundColor(QColor(128, 128, 128));
    _need_redraw = true;
  }

  void LoopCloserViewer::keyPressEvent(QKeyEvent* e) {
    // Visualize all local maps
    if((e->key() == Qt::Key_V)) { 
      for(MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) { _selected_objects.insert(it->get()); }
      _need_redraw = true;
    }
    // Clear viewer from local maps
    else if((e->key() == Qt::Key_C)) {
      _last_relation.reset();
      _selected_objects.clear();
      _new_relations.clear();      
      _candidate_closures.clear();
      _closures.clear();
      _current_local_map = 0;
      _candidate_local_map = 0;
      _selected_local_map = 0;
      _need_redraw = true;
    }
    else if((e->key() == Qt::Key_O)) { 
      _bridge.psToG2o(relations, nodes);
      _bridge.quietOptimize(_graph_optimization_itearations);
      _bridge.g2oToPs(nodes);      
      _need_redraw = true;
    }
    // Geometry check
    if((e->key() == Qt::Key_G)) {
      LocalMap* reference = dynamic_cast<LocalMap*>(_selected_local_map);
      LocalMap* current = dynamic_cast<LocalMap*>(_candidate_local_map);
      if(!reference || !current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to do the ICP local map alignment you need to have exactly two selected local maps... I do nothing" << endl;
  	return;
      }
      TrajectoryMatcherLoopCloser* lcloser = dynamic_cast<TrajectoryMatcherLoopCloser*>(_loop_closer);
      if(!lcloser) { 
	cout << "[WARNING]: the loop closer is not a TrajectoryMatcherLoopCloser... I do nothing" << endl;
	return;
      }
      Eigen::Isometry3f ig = reference->transform().inverse() * current->transform();
      std::tr1::shared_ptr<BinaryNodeRelation> relation;
      lcloser->geometryCheck(relation, reference, current, ig);
    }
    // Start feeding local maps
    else if((e->key() == Qt::Key_F)) { _start_to_feed = true; }
    // Stop feeding local maps
    else if((e->key() == Qt::Key_D)) { _start_to_feed = false; }
    // Align the two local maps selected with ICP
    if((e->key() == Qt::Key_M)) {
      LocalMap* reference = dynamic_cast<LocalMap*>(_selected_local_map);
      LocalMap* current = dynamic_cast<LocalMap*>(_candidate_local_map);
      if(!reference || !current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to do the ICP local map alignment you need to have exactly two selected local maps... I do nothing" << endl;
  	return;
      }
      if(_last_relation) { _last_relation.reset(); }
      _candidate_local_map->push();
      Eigen::Isometry3f initialGuess = reference->transform().inverse() * current->transform();
      _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>(_loop_closer->matchLocalMaps(*reference, 
											     *current,
											     initialGuess));
      _candidate_local_map->setTransform(_selected_local_map->transform() * _last_relation->transform());
      _need_redraw = true;
    }
    // Find loop closures of the current selected local map 
    if((e->key() == Qt::Key_L)) {
      LocalMap* lmap = dynamic_cast<LocalMap*>(_selected_local_map);
      if(!lmap || _selected_objects.size() != 1) {
	cout << "[WARNING]: to search for loop closure you need to have exactly one selected object... I do nothing" << endl;
	return;
      }
      if(!_loop_closer) { 
	cout << "[WARNING]: the loop closer is not setted... I do nothing" << endl;
	return;
      }
      
      _new_relations.clear();      
      _candidate_closures.clear();
      _closures.clear();
      std::cerr << "[DEBUG]: hand " << lmap << " transform is " << t2v(lmap->transform()).transpose() << std::endl; 
      _loop_closer->compute(_candidate_closures, _closures, _new_relations, lmap, &nodes);
      _need_redraw = true;
    }    
    // Save current graph
    else if((e->key() == Qt::Key_S)) {
      if(_output_filename == "" || !_serializable_objects) { 
	std::cerr << "[WARNING]: output filename not given in the command line when this was started, I do nothing" << std::endl; 
	return;
      }
	
      Serializer ser;
      ser.setFilePath(_output_filename);
      ser.setBinaryPath(_output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
      std::cout << "[INFO]: saving " << _serializable_objects->size() << " objects" << std::endl;
      for(std::list<Serializable*>::iterator it = _serializable_objects->begin(); 
	  it != _serializable_objects->end(); 
	  ++it) {
	Serializable* s = *it;
	ser.writeObject(*s);
      }
    }
  }

  void LoopCloserViewer::findLoopClosures(MapNode* map_node) {
    LocalMap* lmap = dynamic_cast<LocalMap*>(map_node);
    if(!_loop_closer) { 
      cout << "[WARNING]: the loop closer is not setted... I do nothing" << endl;
      return;
    }
    if(!lmap) {
      cout << "[WARNING]: received a wrong local map... I do nothing" << endl;
      return;
    }
    
    _bridge.psToG2o(relations, nodes);
    _bridge.quietOptimize(_graph_optimization_itearations);
    _bridge.g2oToPs(nodes);    
    updateGL();
    _new_relations.clear();      
    _candidate_closures.clear();
    _closures.clear();
    std::cerr << "[DEBUG]: auto " << lmap << " transform is " << t2v(lmap->transform()).transpose() << std::endl; 
    std::cerr << "[INFO]: processing local map " << lmap << std::endl;
    std::cerr << "[INFO]: current map has " << nodes.size() << " nodes " << std::endl;
    std::cerr << "[INFO]: current map has " << relations.size() << " relations " << std::endl;
    _loop_closer->compute(_candidate_closures, _closures, _new_relations, lmap, &nodes);
    if(_serializable_objects) { 
      for(BinaryNodeRelationSet::iterator it = _new_relations.begin();
	  it != _new_relations.end();
	  ++it) { _serializable_objects->push_back(it->get()); }
    }
    relations.insert(_new_relations.begin(), _new_relations.end());
    _need_redraw = true;     
  }

  void LoopCloserViewer::postSelection(const QPoint&) {
    int id = selectedName();
    if(id < 0) { return; }
    MapNode* node = _names_map[id];
    if(_selected_objects.count(node)) { 
      _selected_objects.erase(node); 
      if(_selected_local_map == node) { _selected_local_map = 0; }
      else if(_candidate_local_map == node) { _candidate_local_map = 0; }
      std::cerr << "[INFO]: deselected " << node << " object ID " << id << " --- " << t2v(node->transform()).transpose() << std::endl;
    }
    else { 
      _selected_objects.insert(node); 
      if(_selected_local_map == 0) { _selected_local_map = node; }
      else if(_candidate_local_map == 0) { _candidate_local_map = node; }
      std::cerr << "[INFO]: selected " << node << " object ID " << id << " --- " << t2v(node->transform()).transpose() << std::endl;

    }
  }
  
}
