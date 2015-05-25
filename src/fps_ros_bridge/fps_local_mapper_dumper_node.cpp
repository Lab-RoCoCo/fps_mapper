#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include "cloud_publisher_trigger.h"
#include <ros/ros.h>
#include "txt_io/message_writer.h"
#include "local_map_listener.h"
#include "boss/serializer.h"
#include "globals/system_utils.h"

using namespace boss;
using namespace fps_mapper;
using namespace std;
  

class LocalMapDumper: public LocalMapListener {
public:
  LocalMapDumper(boss::Serializer* ser) :
    LocalMapListener(ser){
    _ser= ser;
  }

  virtual void onNewLocalMap(LocalMap* lmap) {
    cerr << "got local map " << lmap->getId() <<endl;
    for ( MapNodeList::iterator it=lmap->nodes().begin(); 
	 it!=lmap->nodes().end(); it++){
      _ser->writeObject(*it->get());
    }
    for (BinaryNodeRelationSet::iterator it=lmap->relations().begin(); 
	 it!=lmap->relations().end(); it++){
      _ser->writeObject(*it->get());
    }
    _ser->writeObject(*lmap);
  }

  virtual void onNewNode(MapNode * n) {
    cerr << "got new node " << n->getId() <<endl;
    _ser->writeObject(*n);
  }

  virtual void onNewRelation(BinaryNodeRelation * r) {
    cerr << "got new relation " << r->getId() <<endl;
    _ser->writeObject(*r);
  }

  virtual void onNewCameraInfo(BaseCameraInfo * cam) {
    cerr << "got new camera info " << cam->getId() << endl;
    _ser->writeObject(*cam);
  }
  boss::Serializer* _ser;
};



const char* banner[]= {
  "fps_local_mapper_dumper_node: dumps incrementally the local maps made by local mapper",
  "usage:",
  " fps_local_mapper_dumper_node <filename>",
  0
};

int main(int argc, char** argv) {
  if (argc<2 || ! strcmp(argv[1],"-h")){
    system_utils::printBanner(banner);
    return 0;
  }
  std::string output_filename = argv[1];
  ros::init(argc, argv, "fps_local_mapper_dumper_node");
  boss::Serializer * ser = new boss::Serializer;
  ser->setFilePath(output_filename);
  ser->setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
  LocalMapDumper* dumper = new LocalMapDumper(ser);
  ros::NodeHandle n;
  dumper->init(n);
  ros::spin();
}
