#include <fstream>
#include <queue> 

#include "globals/system_utils.h"
#include "core/cloud.h"
#include "core/nn_aligner.h"
#include "fps_map/image_map_node.h"
#include "fps_map/local_map.h"
#include "fps_map_viewers/loop_closer_viewer.h"
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
  "fps_loop_closer: does loop closure automatically",
  "usage:",
  " fps_loop_closer [options] input_log.txt",
  " where: ",
  "  -candidates_ratio                 [float] , trajectory morphing max candidate ratio for which a piece of trajectory is not considered informative, default: 0.01",
  "  -max_energy                       [float] , max energy allowed to morph a trajectory over an other, default: 0.001",
  "  -max_distance                     [float] , max distance for which a local map is considered a candidate closure, default: 3.0",
  "  -max_bad_points_distance          [float] , max distance for which two points are considered outliers, default: 0.3",
  "  -max_bad_points_ratio             [float] , max outliers ratio for which a loop closure is discarded, default: 0.1",
  "  -max_neighbors                    [float] , max allowed neighbors local maps, default: 5",
  "  -iterations                       [int]   , iteration for the local maps alignment, default: 10",
  "  -o                                [string], output filename where to write the graph with the closures",
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
  size_t max_neighbors = 5;
  std::string output_filename = "";
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
    else if(!strcmp(argv[c], "-max_neighbors")) {
      c++;
      max_neighbors = atoi(argv[c]);
      std::cout << "max_neighbors: " << max_neighbors << std::endl;
    } 
    else if(!strcmp(argv[c], "-o")) {
      c++;
      output_filename = std::string(argv[c]);
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
  lcloser.setMaxNeighbors(max_neighbors);
  LoopCloserViewer viewer(&lcloser, output_filename, &objects);
  
  std::queue<LocalMap*> local_maps_to_process;
  LocalMap* previous_local_map = 0;
  viewer.show();
  viewer.init();
  bool first_local_map = true;
  while(viewer.isVisible()) {
    app.processEvents();
    if(viewer.needRedraw()) { viewer.updateGL(); }
    if(!viewer.startToFeed()) { usleep(10000); }
    else if((o = des.readObject())) {
      objects.push_back(o);      
      LocalMap* lmap = dynamic_cast<LocalMap*>(o);
      if(lmap) { local_maps_to_process.push(lmap); }
      BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*>(o);
      if(rel) { 
	viewer.relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel)); 
	if(!local_maps_to_process.empty()) {
	  LocalMap* lmap = local_maps_to_process.front();

	  std::cerr << "[INFO]: local maps pair " << previous_local_map << " --- " << lmap << std::endl;
	  if(first_local_map) {
	    viewer.nodes.addElement(lmap);
	    viewer.setCurrentLocalMap(lmap);
	    previous_local_map = lmap;
	    std::cout << "[INFO]: processing " << lmap << std::endl; 		
	    local_maps_to_process.pop();
	    first_local_map = false;
	  }
	  else {
	    BinaryNodeRelationSet::iterator it;
	    for(it = viewer.relations.begin(); it != viewer.relations.end(); ++it) {
	      BinaryNodeRelation* rel = it->get();
	      if(rel->from() == previous_local_map && rel->to() == lmap) { 
		viewer.nodes.addElement(lmap); 
		std::cout << "[INFO]: processing " << lmap << std::endl; 		
		viewer.setCurrentLocalMap(lmap);
		previous_local_map = lmap;
		local_maps_to_process.pop();
		break; 		
	      }
	    }
	  }
	}
      }
    }
    else { usleep(10000); }
  }

  std::cout << "[INFO]: saving the graph" << std::endl;
  if(output_filename == "") { 
    std::cerr << "[WARNING]: output filename not given in the command line when this was started, I do nothing" << std::endl; 
  }
  else {
    Serializer ser;
    ser.setFilePath(output_filename);
    ser.setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    for(std::list<Serializable*>::iterator it = objects.begin(); 
	it != objects.end(); 
	++it) {
      Serializable* s = *it;
      ser.writeObject(*s);
    }
  }
  return 0;
}
