#include "loop_closer_app_viewer.h"

#include "loop_closer/trajectory_matcher_loop_closer.h"

using namespace boss;
using namespace fps_mapper;
using namespace std;
using namespace GLHelpers;

namespace fps_mapper {  

  LoopCloserAppViewer::LoopCloserAppViewer(BaseLoopCloser* loop_closer_): TrajectoryViewer() {
    _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
    _reference = _current = 0;
    _loop_closer = loop_closer_;
  }
  
  void LoopCloserAppViewer::draw() {
    int attrs = ATTRIBUTE_SHOW;    
    attrs |= ATTRIBUTE_SELECTED;


    if(_reference && _candidate_closures.size()) { 
      glColor3f(0, 0, 1);
      _reference->draw(attrs);
    }
    for(BinaryNodeRelationSet::iterator it = _new_relations.begin(); it != _new_relations.end(); ++it) {      
      it->get()->to()->setTransform(it->get()->from()->transform() * it->get()->transform());
      glColor3f(0, 1, 0);
      it->get()->to()->draw(attrs);
    }    
    for(MapNodeList::iterator it = _candidate_closures.begin(); it != _candidate_closures.end(); ++it) {      
      glColor3f(1, 0, 0);
      it->get()->draw(attrs);
    }    

    TrajectoryViewer::draw();
  }

  void LoopCloserAppViewer::init() {
    SimpleViewer::init();
    setAxisIsDrawn(true);
    setBackgroundColor(QColor(128, 128, 128));
    update();
  }

  void LoopCloserAppViewer::postSelection(const QPoint&) {
    int id = selectedName();
    if(id < 0) { return; }
    MapNode* node = _names_map[id];
    LocalMap* lmap = dynamic_cast<LocalMap*>(node);
    if(_selected_objects.count(node)) { 
      _selected_objects.erase(node); 
      if(_reference == node) { _reference = 0; }
      else if(_current == node) { _current = 0; }
      std::cerr << "[INFO]: deselected " << node << " object ID " << id << std::endl;
    }
    else { 
      _selected_objects.insert(node); 
      if(_reference == 0) { _reference = (lmap != 0) ? lmap : 0; }
      else if(_current == 0) { _current = (lmap != 0) ? lmap : 0; }
      std::cerr << "[INFO]: selected " << lmap << " object ID " << id << std::endl;
    }
  }

  void LoopCloserAppViewer::keyPressEvent(QKeyEvent* e) {
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
      _reference = _current = 0;
      update();
    }
    // Add an edge accepting current alignment 
    else if((e->key() == Qt::Key_A)) {
      if(_last_relation) {
	relations.insert(_last_relation);
	_last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
	update();
      }
      for(BinaryNodeRelationSet::iterator it = _new_relations.begin(); it != _new_relations.end(); ++it) {      
	relations.insert(*it);
      }
    }
    // Optimize the graph
    else if((e->key() == Qt::Key_O)) {
      _bridge.psToG2o(relations, nodes);
      _bridge.optimize();
      _bridge.g2oToPs(nodes);
      update();
    }
    // Restore last transform
    if((e->key() == Qt::Key_B)) {
      if(_current) { 
	_current->pop();
	update();
      }
    }
    // Geometry check
    if((e->key() == Qt::Key_G)) {
      if(!_reference || !_current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to do the ICP local map alignment you need to have exactly two selected objects... I do nothing" << endl;
  	return;
      }
      TrajectoryMatcherLoopCloser* lcloser = dynamic_cast<TrajectoryMatcherLoopCloser*>(_loop_closer);
      if(!lcloser) { 
	cout << "[WARNING]: the loop closer is not a TrajectoryMatcherLoopCloser... I do nothing" << endl;
	return;
      }
      Eigen::Isometry3f ig = _reference->transform().inverse() * _current->transform();
      std::tr1::shared_ptr<BinaryNodeRelation> relation;
      lcloser->geometryCheck(relation, _reference, _current, ig);
      update();
    }
    // Apply initial guess to the current local map 
    else if((e->key() == Qt::Key_I)) {
      if(!_reference || !_current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to apply an initial guess you need to have exactly two selected objects... I do nothing" << endl;
  	return;
      }

      _current->push();
      Eigen::Isometry3f delta_t;
      delta_t.translation() = _reference->transform().translation() - _current->transform().translation();
      delta_t.linear() = Eigen::Matrix3f::Identity();
      _current->setTransform(delta_t * _current->transform());
      update();
    }
    // Align the two local maps selected with ICP
    if((e->key() == Qt::Key_M)) {
      if(!_reference || !_current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to do the ICP local map alignment you need to have exactly two selected objects... I do nothing" << endl;
  	return;
      }
      if(_last_relation) { _last_relation.reset(); }
      _current->push();
      Eigen::Isometry3f initialGuess = _reference->transform().inverse() * _current->transform();
      _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>(_loop_closer->matchLocalMaps(*_reference, 
											     *_current,
											     initialGuess));
      _current->setTransform(_reference->transform() * _last_relation->transform());
      update();
    }
    // Align the two trajectories selected with ICP
    if((e->key() == Qt::Key_S)) { 
      if(!_reference || !_current || _selected_objects.size() != 2) {
	cout << "[WARNING]: to do the ICP trajectory alignment you need to have exactly two selected objects... I do nothing" << endl;
  	return;
      }
      TrajectoryMatcherLoopCloser* lcloser = dynamic_cast<TrajectoryMatcherLoopCloser*>(_loop_closer);
      if(!lcloser) { 
	cout << "[WARNING]: the loop closer is not a TrajectoryMatcherLoopCloser... I do nothing" << endl;
	return;
      }
      cout << "ICP(" << _reference << ", " << _current << ")" << std::endl;
      TrajectoryMatcher& tmatcher = lcloser->trajectoryMatcher();
      tmatcher.setReferenceLocalMap(_reference);
      tmatcher.setCurrentLocalMap(_current);

      _current->push();
      Eigen::Isometry3f delta_t = _reference->transform().inverse() * _current->transform();
      delta_t.translation() = Eigen::Vector3f::Zero();
      tmatcher.align(delta_t);
      _current->setTransform(_reference->transform() * tmatcher.T());
      update();      
    }
    // Compute energy between the trajectories of the two selected local maps
    if((e->key() == Qt::Key_E)) {
      if(!_reference || !_current || _selected_objects.size() != 2) {
	cout << "[WARNING]: the selected objects are not valid... I do nothing" << endl;
	return;
      }
      TrajectoryMatcherLoopCloser* lcloser = dynamic_cast<TrajectoryMatcherLoopCloser*>(_loop_closer);
      if(!lcloser) { 
	cout << "[WARNING]: the loop closer is not a TrajectoryMatcherLoopCloser... I do nothing" << endl;
	return;
      }
      Eigen::Isometry3f T;
      cout << "Energy(" << _reference << ", " << _current << "): " 
       	   << lcloser->matchTrajectories(_reference, _current, T) << endl;  
      update();
    }
    // Compute energy between the trajectories of the two selected local maps
    if((e->key() == Qt::Key_L)) {
      if(!_reference || _selected_objects.size() != 1) {
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
      _loop_closer->compute(_candidate_closures, _closures, _new_relations, _reference, &nodes);
      update();
    }    
  }
  
}
