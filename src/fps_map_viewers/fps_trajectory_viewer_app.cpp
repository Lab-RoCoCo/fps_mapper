#include <fstream>
#include "globals/system_utils.h"
#include "core/cloud.h"
#include "fps_map/image_map_node.h"
#include "fps_map/local_map.h"
#include "trajectory_viewer.h"
#include "qapplication.h"
#include <stdexcept>
#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"

using namespace fps_mapper;
using namespace boss;
using namespace std;

BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
BinaryNodeRelation rel;

const char* banner[]= {
  "fps_trajectory_viewer_app: example on how to show load a set of boss objects constituting a boss map",
  "usage:",
  " fps_trajectory_viewer_app  <boss filename>",
  0
};

int main (int argc, char** argv) {
  if (argc<2 || ! strcmp(argv[1],"-h")) {
    system_utils::printBanner(banner);
    return 0 ;
  }
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[1]);
  Serializable* o;
  MapNodeList lmaps;
  while ( (o = des.readObject()) ){
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if (lmap)
      lmaps.addElement(lmap);
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  cerr << "Read: " << lmaps.size() << " local maps" << endl;

  QApplication app(argc, argv);
  TrajectoryViewer viewer;
  viewer.nodes = lmaps;
  viewer.show();
  app.exec();
  return 1;
}
