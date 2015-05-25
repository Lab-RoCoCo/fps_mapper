#include <iostream>
#include "fps_map/image_map_node.h"
#include "fps_map/multi_image_map_node.h"
#include "local_map_triggers.h"
#include "tracker/multi_tracker.h"

namespace fps_mapper {

  TrajectoryMakerTrigger::TrajectoryMakerTrigger(Tracker* tracker,
						 int events,
						 int priorory):
    Tracker::Trigger(tracker, events,  priorory){
    cerr<< " name: TrajectoryMakerTrigger "<< endl;
    _trajectory_min_translation = 0.05;
    _trajectory_min_orientation = 0.1;
    _last_global_pose = _tracker->globalT();
    }

  void TrajectoryMakerTrigger::action(Tracker::TriggerEvent e){
    if (e&Tracker::NEW_CAMERA_ADDED){
      onCameraInfoCreated(tracker()->lastCamera());
      return;
    }
    if ( (e&Tracker::TRACK_GOOD) || (e&Tracker::REFERENCE_FRAME_RESET)) {
      Eigen::Isometry3f delta = _last_global_pose.inverse()*_tracker->globalT();
      Eigen::AngleAxisf aa(delta.linear());
      bool make_new_node = (e&Tracker::REFERENCE_FRAME_RESET) ||
	delta.translation().norm()>_trajectory_min_translation
	|| fabs(aa.angle())>_trajectory_min_orientation;
      if (make_new_node && _nodes) {
	MapNode* new_node = 0;
	MapNode* previous_node = 0;
	if (_nodes->size())
	  previous_node = _nodes->rbegin()->get();
	MultiTracker* mt = dynamic_cast<MultiTracker*>(_tracker);
	if (mt) {
	  MultiCameraInfo* mci = dynamic_cast<MultiCameraInfo*>(_tracker->lastCamera());
	  MultiImageMapNode * n = new MultiImageMapNode(_tracker->globalT(), mci, _tracker->lastTopic(), _tracker->lastSeq());
	  n->subimageSeqs() = mt->lastSeqs();
	  new_node = n;
	  _nodes->addElement(n);
	} else {
	  new_node = new ImageMapNode(_tracker->globalT(), _tracker->lastCamera(), _tracker->lastTopic(), _tracker->lastSeq());
	  _nodes->addElement(new_node);
	}
	_last_global_pose = _tracker->globalT();
	BinaryNodeRelation* rel=0;
	if (_relations && previous_node) {
	  
	  rel = new BinaryNodeRelation;
	  rel->setFrom(previous_node);
	  rel->setTo(new_node);
	  rel->setTransform(previous_node->transform().inverse()*new_node->transform());
	  rel->setInformationMatrix(_tracker->informationMatrix());
	  _relations->insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
	  _tracker->resetInformationMatrix();
	}
	onNewNodeCreated(new_node, rel);
      } 
    }
  }
  
  void TrajectoryMakerTrigger::onCameraInfoCreated(BaseCameraInfo* ){}
  
  void TrajectoryMakerTrigger::onNewNodeCreated(MapNode*, BinaryNodeRelation*){}

  void LocalMapTrigger::onRelationCreated(BinaryNodeRelation* ){}

  LocalMapTrigger::LocalMapTrigger(Tracker* tracker,
				   int events,
				   int priorory,
				   boss::Serializer* ser) :
    TrajectoryMakerTrigger(tracker, events, priorory) {
    cerr<< " name: LocalMapTrigger "<< endl;
    _trajectory_max_translation = 0.25;
    _trajectory_max_orientation = 1;
    _serializer = ser;
    _local_maps = 0;
    _relations = 0;
    _nodes = new MapNodeList;
    _relations = new BinaryNodeRelationSet;
  }
  void LocalMapTrigger::action(Tracker::TriggerEvent e) {
    TrajectoryMakerTrigger::action(e);
    if (e&Tracker::REFERENCE_FRAME_RESET)
      return;

    if (!(e&Tracker::TRACKING_DONE))
      return;

    LocalMap* lmap = 0;
    // check if the trajectory bounds exceeded
    bool bound_exceded = isTrajectoryBoundReached();

    //cerr << "bound exceeded: " << bound_exceded << endl;
    //cerr << "track_broken:  " <<  _tracker->isTrackBroken() << endl;
    //cerr << "nodes:         " <<  _nodes->size() << endl;
    
    if (bound_exceded || _tracker->isTrackBroken()) {
      //cerr << "making local map" << endl;
      lmap  = makeLocalMap();
      onLocalMapCreated(lmap);
      if (_serializer) {
	saveCameras(_tracker->cameras());
	saveLocalMap(*lmap);
      }

      if (_last_local_map) {
	Eigen::Isometry3f dt = _last_local_map->transform().inverse()*lmap->transform();
	Matrix6f info = Matrix6f::Identity();
	if (_tracker->isTrackBroken()) {
	  info *=1e-2;
	} 
	std::tr1::shared_ptr<BinaryNodeRelation> 
	  rel (new BinaryNodeRelation(_last_local_map.get(), lmap, dt, info) );
	onRelationCreated(rel.get());
	if (_serializer)
	  _serializer->writeObject(*rel);

	_last_relation = rel;
	if (_relations)
	  _relations->insert(rel);
      }
      
      std::tr1::shared_ptr<LocalMap> current_map_ptr(lmap);

      if (_local_maps) {
	_local_maps->push_back(current_map_ptr);
      }
      _previous_local_map = _last_local_map;
      _last_local_map = current_map_ptr;
      _nodes->clear();
      _relations->clear();
      if (! _tracker->isTrackBroken() && _tracker->referenceModel())
	_tracker->referenceModel()->clip(_tracker->aligner().maxDistance());
      else
	_tracker->clearStatus();
    }
  }

  LocalMap* LocalMapTrigger::makeLocalMap() {
    if (! _nodes)
      return 0;
    if (! _tracker->referenceModel() || ! _tracker->referenceModel()->size())
      return 0;

    Eigen::Isometry3f origin = _nodes->middlePose();
    Eigen::Isometry3f invT = origin.inverse();
    LocalMap * local_map = new LocalMap(origin);
    local_map->nodes()=*_nodes;
    local_map->relations()=*_relations;
    for (MapNodeList::iterator it = _nodes->begin(); 
	 it!= _nodes->end(); it++) {
      MapNode* node = it->get();
      node->parents().insert(local_map);
      node->setTransform(invT*node->transform());
    }
    for (BinaryNodeRelationSet::iterator it= _relations->begin(); it!=_relations->end(); it++) {
      BinaryNodeRelation* rel = it->get();
      rel->setParent(local_map);
    }
    local_map->setCloud(new Cloud(*_tracker->referenceModel()));
    local_map->cloud()->transformInPlace(invT*_tracker->globalT());
    return local_map;
  }

  void LocalMapTrigger::saveLocalMap(LocalMap& lmap){
    for (MapNodeList::iterator tt = lmap.nodes().begin();
	 tt!=lmap.nodes().end(); tt++){
      MapNode* n = tt->get();
      _serializer->writeObject(*n);
    }

    for (BinaryNodeRelationSet::iterator it = lmap.relations().begin();
	 it!=lmap.relations().end(); it++){
      BinaryNodeRelation* r = it->get();
      _serializer->writeObject(*r);
    }
      
    // write the local map
    _serializer->writeObject(lmap);

  }

  void LocalMapTrigger::onLocalMapCreated(LocalMap* lmap){
    cerr << "Local Map created" << endl;
  }

  void LocalMapTrigger::saveCameras(CameraInfoManager& manager) {
    for(size_t i = 0; i< manager.cameras().size(); i++) {
      BaseCameraInfo* cam = manager.cameras()[i];
      _serializer->writeObject(*cam);
    }
  }


  bool LocalMapTrigger::isTrajectoryBoundReached() {
    if (!_nodes || !_nodes->size())
      return false;
    Eigen::Vector3f tbb = _nodes->upperTranslation() - _nodes->lowerTranslation();
    if (tbb.norm()>_trajectory_max_translation)
      return true;

    Eigen::Vector3f obb = _nodes->upperOrientation() - _nodes->lowerOrientation();
    if (obb.norm()>_trajectory_max_orientation)
      return true;
    return false;
  }



}
