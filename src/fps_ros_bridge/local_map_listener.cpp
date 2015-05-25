#include "local_map_listener.h"
#include <iostream>

namespace fps_mapper {
  using namespace std;

  LocalMapListener::LocalMapListener(boss::IdContext* context_) {
    _context = context_;
    if (! _context)
      _context = new boss::IdContext;
  }

  void LocalMapListener::pinholeCameraInfoCallback(const PinholeCameraInfoMsgConstPtr& msg){
    _pending_camera_info_msgs.push_back(*msg);
    processPendingCameraInfos();
  }

  void LocalMapListener::multiCameraInfoCallback(const MultiCameraInfoMsgConstPtr& msg){
    _pending_multi_camera_info_msgs.push_back(*msg);
    processPendingCameraInfos();
  }

  void LocalMapListener::imageMapNodeCallback(const ImageMapNodeMsgConstPtr& msg){
    _pending_image_node_msgs.push_back(*msg);
    processPendingCameraInfos();
    processPendingImageNodes();
  }

  void LocalMapListener::multiImageMapNodeCallback(const MultiImageMapNodeMsgConstPtr& msg){
   _pending_multi_image_node_msgs.push_back(*msg);
    processPendingCameraInfos();
    processPendingImageNodes();
  }

  void LocalMapListener::localMapCallback(const LocalMapMsgConstPtr& msg){
    _pending_local_map_msgs.push_back(*msg);
    processPendingMsgs();
  }

  void LocalMapListener::relationsCallback(const BinaryNodeRelationMsgConstPtr& msg){
    _pending_relations_msgs.push_back(*msg);
    processPendingRelations();
  }
  
  void LocalMapListener::processPendingRelations() {
    std::list<BinaryNodeRelationMsg>::iterator it = _pending_relations_msgs.begin();
 
    while(it!=_pending_relations_msgs.end()){
     BinaryNodeRelation* rel=msg2binaryNodeRelation(*it,_context);
      if (rel) {
	// check that both the nodes are in the pool
	if(_objects.find(rel->from())!=_objects.end() ||
	   _objects.find(rel->to())!=_objects.end()) {
	  _objects.insert(rel);
	  onNewRelation(rel);
	  std::list<BinaryNodeRelationMsg>::iterator itd=it;
	  it++;
	  _pending_relations_msgs.erase(itd);
	} else
	  it++;
      } else
	it++;
    }
  }

  void LocalMapListener::processPendingCameraInfos(){
    {
      std::list<PinholeCameraInfoMsg>::iterator it = _pending_camera_info_msgs.begin();
      while(it!=_pending_camera_info_msgs.end()){			      
	const PinholeCameraInfoMsg& msg = *it;
	PinholeCameraInfo* cam=msg2pinholeCameraInfo(msg,_context);
	if (cam) {
	  _objects.insert(cam);
	  onNewCameraInfo(cam);
	  std::list<PinholeCameraInfoMsg>::iterator itd=it;
	  it++;
	  _pending_camera_info_msgs.erase(itd);
	} else
	  it++;
      }
    }
    {
      std::list<MultiCameraInfoMsg>::iterator it = _pending_multi_camera_info_msgs.begin();
      while(it!=_pending_multi_camera_info_msgs.end()){			      
	const MultiCameraInfoMsg& msg = *it;
	MultiCameraInfo* cam=msg2multiCameraInfo(msg,_context);
	if (cam) {
	  _objects.insert(cam);
	  onNewCameraInfo(cam);
	  std::list<MultiCameraInfoMsg>::iterator itd=it;
	  it++;
	  _pending_multi_camera_info_msgs.erase(itd);
	} else
	  it++;
      }
    }
  }

  void LocalMapListener::processPendingImageNodes(){
    {
      std::list<ImageMapNodeMsg>::iterator it = _pending_image_node_msgs.begin();
      while(it!=_pending_image_node_msgs.end()){			      
	const ImageMapNodeMsg& msg = *it;
	ImageMapNode* node=msg2imageMapNode(msg,_context);
	if (node) {
	  _objects.insert(node);
	  onNewNode(node);
	  std::list<ImageMapNodeMsg>::iterator itd = it;
	  it++;
	  _pending_image_node_msgs.erase(itd);
	} else
	  it++;
      }
    }

    {
      std::list<MultiImageMapNodeMsg>::iterator it = _pending_multi_image_node_msgs.begin();
      while(it!=_pending_multi_image_node_msgs.end()){			      
	const MultiImageMapNodeMsg& msg = *it;
	MultiImageMapNode* node=msg2multiImageMapNode(msg,_context);
	if (node) {
	  _objects.insert(node);
	  onNewNode(node);
	  std::list<MultiImageMapNodeMsg>::iterator itd = it;
	  it++;
	  _pending_multi_image_node_msgs.erase(itd);
	} else
	  it++;
      }
    }
  }

  void LocalMapListener::processPendingLocalMaps() {
    std::list<LocalMapMsg>::iterator it = _pending_local_map_msgs.begin();
    while (it!=_pending_local_map_msgs.end()){
      const LocalMapMsg& msg = *it;
      LocalMap* lmap = msg2localMap(msg, _context);
      if (lmap) {
	_objects.insert(lmap);
	onNewLocalMap(lmap);
	std::list<LocalMapMsg>::iterator itd = it;
	it++;
	_pending_local_map_msgs.erase(itd);
      } else
	it++;
    }
  }

  void LocalMapListener::processPendingMsgs(){
    processPendingCameraInfos();
    processPendingImageNodes();
    processPendingRelations();
    processPendingLocalMaps();
  }



  void LocalMapListener::init(ros::NodeHandle& n){
    _sub_camera_info = n.subscribe<PinholeCameraInfoMsg>("/local_mapper/camera_info", 10, boost::bind(&LocalMapListener::pinholeCameraInfoCallback, this, _1));
    _sub_multi_camera_info = n.subscribe<MultiCameraInfoMsg>("/local_mapper/multi_camera_info", 10, boost::bind(&LocalMapListener::multiCameraInfoCallback, this, _1));
    _sub_image_map_node = n.subscribe<ImageMapNodeMsg>("/local_mapper/image_node", 10000, boost::bind(&LocalMapListener::imageMapNodeCallback, this, _1));
    _sub_multi_image_map_node = n.subscribe<MultiImageMapNodeMsg>("/local_mapper/multi_image_node", 1000, boost::bind(&LocalMapListener::multiImageMapNodeCallback, this, _1));
    _sub_local_map = n.subscribe<LocalMapMsg>("/local_mapper/local_map", 10, boost::bind(&LocalMapListener::localMapCallback, this, _1));
    _sub_relations = n.subscribe<BinaryNodeRelationMsg>("/local_mapper/relations", 10000, boost::bind(&LocalMapListener::relationsCallback, this, _1));
  }

  void LocalMapListener::onNewLocalMap(LocalMap* lmap){
    cerr << "got new local map, id:" << lmap->getId() << endl;
  }

  void LocalMapListener::onNewNode(MapNode* node){
    cerr << "got new node, id: " << node->getId() << endl;
  }

  void LocalMapListener::onNewRelation(BinaryNodeRelation* rel) {
    cerr << "got new relation  , id: " << rel->getId() << endl;
  }

  void LocalMapListener::onNewCameraInfo(BaseCameraInfo* cam) {
    cerr << "got new camera info  , id: " << cam->getId() << endl;
  }


}
