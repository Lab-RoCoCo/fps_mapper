#include <fstream>
#include "image_map_node.h"
#include "local_map.h"

#include "globals/system_utils.h"
#include "qapplication.h"
#include <stdexcept>
#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"

using namespace fps_mapper;
using namespace boss;
using namespace std;

const char* banner[]= {
  "fps_trajectory_loader_app: ecample on how to load a set of boss objects",
  "usage:",
  " fps_trajectory_loader_app  <boss filename>",
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
  while ( (o = des.readObject()) ){
    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  return 1;
}
