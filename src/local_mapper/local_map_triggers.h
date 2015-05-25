#pragma once
#include "fps_map/local_map.h"
#include "fps_map/binary_node_relation.h"
#include "boss/serializer.h"
#include "tracker/tracker.h"

namespace fps_mapper {

  class TrajectoryMakerTrigger: public Tracker::Trigger{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    TrajectoryMakerTrigger(Tracker* tracker,
			   int events = 
			   Tracker::TRACK_GOOD|
			   Tracker::REFERENCE_FRAME_RESET|
			   Tracker::NEW_CAMERA_ADDED,
			   int priorory = 10);
    virtual void action(Tracker::TriggerEvent e);
    MapNodeList* nodes() {return _nodes;}
    BinaryNodeRelationSet* relations() { return _relations;}
    void setNodes(MapNodeList* nodes_) {_nodes = nodes_;}

    // min translation between two trajectory nodes
    inline float trajectoryMinTranslation()  const { return _trajectory_min_translation; }
    inline void setTrajectoryMinTranslation(float v)  { _trajectory_min_translation = v; }

    // min orientation between two trajectory nodes
    inline float trajectoryMinOrientation()  const { return _trajectory_min_orientation; }
    inline void setTrajectoryMinOrientation(float v)  { _trajectory_min_orientation = v; }
    virtual void onNewNodeCreated(MapNode* node, BinaryNodeRelation* rel=0);
    virtual void onCameraInfoCreated(BaseCameraInfo* camera_info);
  protected:
    Eigen::Isometry3f _last_global_pose;
    MapNodeList* _nodes;
    BinaryNodeRelationSet* _relations;
    float _trajectory_min_translation;
    float _trajectory_min_orientation;
  };


  class LocalMapTrigger: public TrajectoryMakerTrigger {
  public:
    LocalMapTrigger(Tracker* tracker,
		    int events = 
		    Tracker::TRACK_GOOD|
		    Tracker::TRACK_BROKEN|
		    Tracker::REFERENCE_FRAME_RESET|
		    Tracker::TRACKING_DONE|
		    Tracker::NEW_CAMERA_ADDED,
		    int priorory = 10,
		    boss::Serializer* ser=0);
    virtual void action(Tracker::TriggerEvent e);
    virtual void onLocalMapCreated(LocalMap* lmap);
    virtual void onRelationCreated(BinaryNodeRelation* rel);

    
    inline boss::Serializer* serializer() const {return _serializer;}
    inline void setSerializer(boss::Serializer* ser) {_serializer = ser;}

    inline MapNodeList* localMaps() { return _local_maps; }
    inline void setLocalMaps(MapNodeList* local_maps) { _local_maps = local_maps; }

    inline BinaryNodeRelationSet* localMapsRelations() { return _relations; }
    inline void setLocalMapsRelations(BinaryNodeRelationSet* relations) { _relations = relations; }


    // max size of the bounding box of the trajectory translation, after which a new local map is created
    inline float trajectoryMaxTranslation()  const { return _trajectory_max_translation; }
    inline void setTrajectoryMaxTranslation(float v)  { _trajectory_max_translation = v; }

    // max size of the bounding box of the trajectory orientation, after which a new local map is created
    inline float trajectoryMaxOrientation()  const { return _trajectory_max_orientation; }
    inline void setTrajectoryMaxOrientation(float v)  { _trajectory_max_orientation = v; }


    bool isTrajectoryBoundReached();

  protected:
    LocalMap* makeLocalMap();
    void saveLocalMap(LocalMap& lmap);
    void saveCameras(CameraInfoManager& manager);

    std::tr1::shared_ptr<LocalMap> _last_local_map, _previous_local_map;
    std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
    bool _enable;
    boss::Serializer* _serializer;
    float _trajectory_max_translation;
    float _trajectory_max_orientation;
    MapNodeList* _local_maps;
    //BinaryNodeRelationSet* _relations;
  };

}

