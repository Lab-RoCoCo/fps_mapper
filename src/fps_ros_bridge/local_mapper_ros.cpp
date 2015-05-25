#include "local_mapper_ros.h"

namespace fps_mapper{

  void LocalMapperRos::init(ros::NodeHandle& nh) {
    _image_node_pub = nh.advertise<fps_mapper::ImageMapNodeMsg>("/local_mapper/image_node", 10000);
    _multi_image_node_pub = nh.advertise<fps_mapper::MultiImageMapNodeMsg>("/local_mapper/multi_image_node", 1000);
    _camera_info_pub = nh.advertise<fps_mapper::PinholeCameraInfoMsg>("/local_mapper/camera_info", 100);
    _multi_camera_info_pub = nh.advertise<fps_mapper::MultiCameraInfoMsg>("/local_mapper/multi_camera_info", 10);
    _local_map_pub = nh.advertise<fps_mapper::LocalMapMsg>("/local_mapper/local_map",20);
    _relations_pub = nh.advertise<fps_mapper::BinaryNodeRelationMsg>("/local_mapper/relations",10000);
  }

  void LocalMapperRos::serializeCameras() {
    //cerr << endl << "serializing cameras" << endl << endl;
    for (size_t i = 0; i<_camera_infos.size(); i++){
      //cerr << i << endl;
      BaseCameraInfo* cam = _camera_infos[i];
      serializeCamera(cam);
    }
  }

  void LocalMapperRos::serializeCamera(BaseCameraInfo* cam) {
    //cerr << cam->className() << endl;
    PinholeCameraInfo* pci = dynamic_cast<PinholeCameraInfo*>(cam);
    if (pci) {
      PinholeCameraInfoMsg msg = pinholeCameraInfo2msg(pci, _context);
      _camera_info_pub.publish(msg);
      //BaseCameraInfo* camCopy = msg2pinholeCameraInfo(msg, &copy_context);
    }
    MultiCameraInfo* mci = dynamic_cast<MultiCameraInfo*>(cam);
    if (mci) {
      for (size_t i = 0; i<mci->cameraInfos().size(); i++){
	serializeCamera(mci->cameraInfos()[i]);
      }
      MultiCameraInfoMsg msg = multiCameraInfo2msg(mci, _context);
      _multi_camera_info_pub.publish(msg);
      //BaseCameraInfo* camCopy = msg2multiCameraInfo(msg, &copy_context);
    }
  }


  void LocalMapperRos::serializeTrajectory(MapNodeList& nodes) {
    //cerr << endl << "serializing trajectory" << endl << endl;
    // serialize the trajectory
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++){
      MapNode* n = it->get();
      serializeNode(n);
    }
    //cerr << endl << "done " << endl << endl;
  }

  void LocalMapperRos::serializeNode(MapNode* n) {
    ImageMapNode* imn = dynamic_cast<ImageMapNode*>(n);
    if(imn) {
      ImageMapNodeMsg msg = imageMapNode2msg(imn, _context);
      _image_node_pub.publish(msg);
      //MapNode* n = msg2imageMapNode(msg, &copy_context);
    }
    MultiImageMapNode* mmn = dynamic_cast<MultiImageMapNode*>(n);
    if(mmn) {
      MultiImageMapNodeMsg msg = multiImageMapNode2msg(mmn, _context);
      _multi_image_node_pub.publish(msg);
      //MapNode* n = msg2multiImageMapNode(msg, &copy_context);
    }
  }


  void LocalMapperRos::serializeRelation(BinaryNodeRelation* rel) {
    BinaryNodeRelationMsg msg = binaryNodeRelation2msg(rel, _context);
    _relations_pub.publish(msg);
  }

  void LocalMapperRos::onRelationCreated(BinaryNodeRelation* rel){
    serializeRelation(rel);
  }


  void LocalMapperRos::onNewNodeCreated(MapNode* n, BinaryNodeRelation* rel) {
    n->setTimestamp(tracker()->lastTimestamp());
    serializeNode(n);
    if (rel) {
      serializeRelation(rel);
    }
  }

  void LocalMapperRos::onLocalMapCreated(LocalMap* lmap){
    serializeCameras();
    serializeTrajectory(lmap->nodes());
    for (BinaryNodeRelationSet::iterator it = lmap->relations().begin(); it!=lmap->relations().end(); it++)
      serializeRelation(it->get());
    cerr << "New local map boradcasted, object id: " << lmap->getId() << endl;
    LocalMapMsg mapmsg = localMap2msg(lmap, _context);
    _local_map_pub.publish(mapmsg);
    //LocalMap * mapCopy = msg2localMap(mapmsg, &copy_context);
  }

  void LocalMapperRos::onCameraInfoCreated(BaseCameraInfo* camera_info){
    //cerr << "got new camera info" << endl;
    _camera_infos.push_back(camera_info);
    serializeCamera(camera_info);
  }
}
