#include "ros_utils.h"
#include "ros_msgs.h"
#include <iostream>

using namespace fps_mapper;
using namespace std;

MapNode* n = new MapNode;

int main(int argc, char** argv){
  boss::IdContext context1;
  boss::IdContext context2;

  std::vector<PinholeCameraInfo*> camera_infos;
  std::vector<BaseCameraInfo*> base_camera_infos;

  for (int i = 0; i<10; i++) {
    PinholeCameraInfo* c = new PinholeCameraInfo;
    camera_infos.push_back(c);
    base_camera_infos.push_back(c);
  }

  MultiCameraInfo* multi_cam = new MultiCameraInfo;
  multi_cam->cameraInfos()=base_camera_infos;


  for (int i = 0; i<camera_infos.size(); i++) {
    PinholeCameraInfo* cam1 = camera_infos[i];
    PinholeCameraInfoMsg msg1 = pinholeCameraInfo2msg(cam1, &context1);
    cerr << "msg1.id" << msg1.id << endl;

    BaseCameraInfo* cam2 = msg2pinholeCameraInfo(msg1, &context2);
    cerr << "cam2.id()" << cam2->getId() << endl;
  }
  
  MultiCameraInfoMsg mcmsg = multiCameraInfo2msg(multi_cam, &context1);
  MultiCameraInfo* mci = msg2multiCameraInfo(mcmsg, &context2);


  std::vector<MapNode*> nodes;
  for (size_t i = 0; i< 10; i++) {
    MapNode* n = new MapNode;
    MapNodeMsg msg = mapNode2msg(n, &context1);
    MapNode* n2 = msg2MapNode(msg, &context2);
    nodes.push_back(n2);
  }

  for (size_t i = 0; i< 10; i++) {
    ImageMapNode* n = new ImageMapNode;
    n->setCameraInfo(camera_infos[0]);
    ImageMapNodeMsg msg = imageMapNode2msg(n, &context1);
    ImageMapNode* n2 = msg2imageMapNode(msg, &context2);
    nodes.push_back(n2);
  }

  for (size_t i = 0; i< nodes.size(); i++) {
    MapNode* n = nodes[i];
    cerr << "n: " << n->getId() << endl;
  }

  LocalMap* lmap = new LocalMap;
  for (size_t i = 0; i<nodes.size(); i++) {
    lmap->nodes().addElement(nodes[i]);
  }
  lmap->setCloud(new Cloud);

  LocalMapMsg lmap_msg=localMap2msg(lmap, &context1);
  LocalMap* lmap2 = msg2localMap(lmap_msg, &context2);
  

  for (MapNodeList::iterator it = lmap2->nodes().begin(); it!=lmap2->nodes().end(); it++){
    MapNode* n = it->get();
    cerr << n->getId() << endl;
  }
  
}
