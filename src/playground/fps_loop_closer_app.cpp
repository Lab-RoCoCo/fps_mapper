#include <fstream>
#include "globals/system_utils.h"
#include "core/cloud.h"
#include "core/nn_aligner.h"
#include "fps_map/image_map_node.h"
#include "fps_map/local_map.h"
#include "fps_map_viewers/loop_closer_app_viewer.h"
#include "qapplication.h"
#include "qevent.h"
#include <stdexcept>
#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"
#include "global_optimization/g2o_bridge.h"
#include "loop_closer/trajectory_matcher_loop_closer.h"

using namespace fps_mapper;
using namespace boss;
using namespace std;

const char* banner[] = {
  "fps_loop_closer_app: does loop closure",
  "usage:",
  " fps_loop_closer_app [options] input_log.txt",
  " where: ",
  "  -candidates_ratio                 [float] , trajectory morphing max candidate ratio for which a piece of trajectory is not considered informative, default: 0.01",
  "  -max_energy                       [float] , max energy allowed to morph a trajectory over an other, default: 0.001",
  "  -max_distance                     [float] , max distance for which a local map is considered a candidate closure, default: 3.0",
  "  -max_bad_points_distance          [float] , max distance for which two points are considered outliers, default: 0.3",
  "  -max_bad_points_ratio             [float] , max outliers ratio for which a loop closure is discarded, default: 0.1",
  "  -iterations                       [int]   , iteration for the local maps alignment, default: 10",
  "once the stuff starts",
  "shift-left-click on a node: highlights the local map of the node",  
  "E: compute energy necessary to morph the two selected trajectories",
  "A: accepts the most recent loop closure found (press any other key to discard)",
  "S: superpose two local map trajectories using ICP (if there are only two higlighted)",
  "I: superpose two local map trajectories applying the initial guess (if there are only two higlighted)",
  "B: undo last transform applied to the candidate local map",
  0
};

class RefinerViewer: public TrajectoryViewer{
public:
  RefinerViewer() {}
  void keyPressEvent(QKeyEvent *e){}

protected:
  std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
};

int main (int argc, char** argv) {
  int c = 1;
  bool single = false;
  int iterations = 10;
  float candidates_ratio = 0.01f;
  float max_energy = 0.001f;
  float max_distance = 3.0f;
  float max_bad_points_distance = 0.3f;
  float max_bad_points_ratio = 0.1f;
  std::vector<std::string> topics;
  while(c < argc) {
    if(!strcmp(argv[c], "-h")) {
      system_utils::printBanner(banner);
      return 0;
    } 
    else if(!strcmp(argv[c], "-candidate_ratio")) {
      c++;
      candidates_ratio = atof(argv[c]);
      std::cout << "candidates_ratio: " << candidates_ratio << std::endl;
    }
    else if(!strcmp(argv[c], "-max_energy")) {
      c++;
      max_energy = atof(argv[c]);
      std::cout << "max_energy: " << max_energy << std::endl;
    } 
    else if(!strcmp(argv[c], "-max_distance")) {
      c++;
      max_distance = atof(argv[c]);
      std::cout << "max_distance: " << max_distance << std::endl;
    }
    else if(!strcmp(argv[c], "-max_bad_points_distance")) {
      c++;
      max_bad_points_distance = atof(argv[c]);
      std::cout << "max_bad_points_distance: " << max_bad_points_distance << std::endl;
    } 
    else if(!strcmp(argv[c], "-max_bad_points_ratio")) {
      c++;
      max_bad_points_ratio = atof(argv[c]);
      std::cout << "max_bad_points_ratio: " << max_bad_points_ratio << std::endl;
    } 
    else if(!strcmp(argv[c], "-iterations")) {
      c++;
      iterations = atoi(argv[c]);
      std::cout << "iterations: " << iterations << std::endl;
    } 
    c++;
  }

  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(argv[c - 1]);
  Serializable* o;
  QApplication app(argc, argv);
  NNAligner aligner;
  aligner.finder().setPointsDistance(1.0f);
  aligner.finder().setNormalAngle(M_PI_4);
  aligner.setIterations(iterations);
  TrajectoryMatcherLoopCloser lcloser(&aligner);
  lcloser.setMaxEnergy(max_energy);
  lcloser.setCandidatesRatio(candidates_ratio);
  lcloser.setMaxDistance(max_distance);
  lcloser.setMaxBadPointsDistance(max_bad_points_distance);
  lcloser.setMaxBadPointsRatio(max_bad_points_ratio);
  LoopCloserAppViewer viewer(&lcloser);
  
  while((o = des.readObject())) {
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if(lmap) { viewer.nodes.addElement(lmap); }
    BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
    if(rel) { viewer.relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel)); }
    objects.push_back(o);
  }
  cerr << "[INFO]: read " << objects.size() << " elements" << endl;
  cerr << "[INFO]: read " << viewer.nodes.size() << " local maps" << endl;
  
  viewer.show();
  viewer.init();
  app.exec();

  return 0;
}
