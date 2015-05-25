#include <fstream>
#include "globals/system_utils.h"
#include "core/cloud.h"
#include "core/nn_aligner.h"
#include "fps_map/image_map_node.h"
#include "fps_map/local_map.h"
#include "fps_map_viewers/trajectory_viewer.h"
#include "qapplication.h"
#include "qevent.h"
#include <stdexcept>
#include "boss/deserializer.h"
#include "boss/serializer.h"
#include "boss/trusted_loaders.h"
#include "global_optimization/g2o_bridge.h"

using namespace fps_mapper;
using namespace boss;
using namespace std;

BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
NNAligner aligner;
BinaryNodeRelation rel;



BinaryNodeRelation* matchLocalMaps(LocalMap& reference, LocalMap& current, double voxel_size=0.05, bool reset_trans=true){
  cerr << "Matching" << endl;
  // step 1, determine the transformation between reference and current
  Eigen::Isometry3f T = reference.transform().inverse() * current.transform();
  
  // if reset_trans is true ignore the translation between the two local maps, and set it to zero
  if( reset_trans ) {
    T.translation().setZero();
  }
  // step 2 prepare the aligner

  Cloud tempRef(*reference.cloud());
  Cloud tempCurr(*current.cloud());
  voxelize(tempRef, voxel_size);
  voxelize(tempCurr, voxel_size);
  aligner.finder().setPointsDistance(1.);
  aligner.finder().setNormalAngle(M_PI/4);
  aligner.setIterations(10);
  aligner.setReferenceModel(&tempRef);
  aligner.setCurrentModel(&tempCurr);
  aligner.align(T);
  current.setTransform(reference.transform()*aligner.T());
  BinaryNodeRelation* rel= new BinaryNodeRelation(&reference, &current, aligner.T(), Matrix6f::Identity());
  return rel;
}



G2OBridge bridge;



class RefinerViewer: public TrajectoryViewer{
public:
  RefinerViewer(std::string output_filename_ = "", std::list<Serializable*>* serializable_objects_ = 0) { 
    _output_filename = output_filename_; 
    _serializable_objects = serializable_objects_;
    _voxel_size = 0.05;
    _reset_trans = true;    // by default, we reset the relative translation between two local maps for the matching
  }
  void keyPressEvent(QKeyEvent *e)
  {
    switch( e->key() ) {
      case Qt::Key_M: {     // match maps (if exactly two maps are selected)
        if (_selected_objects.size()!=2) {
          cerr << "to do the matching you need to have exactly twom objects, I do nothing" << endl;
          return;
        }
        LocalMap* reference, *current;
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        reference = dynamic_cast<LocalMap*>(*it);
        it++;
        current = dynamic_cast<LocalMap*>(*it);
        if (! reference || ! current)
        if (_selected_objects.size()!=2) {
          cerr << "invalid object types, I do nothing" << endl;
          return;
        }
        if (_last_relation)
          _last_relation.reset();
        _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>(matchLocalMaps(*reference, *current, _voxel_size, _reset_trans));
        update(); // Refresh display
        break;
      }
      case Qt::Key_O: {     // optimize the graph
        bridge.psToG2o(relations, nodes);
        bridge.optimize();
        bridge.g2oToPs(nodes);
        update(); // Refresh display
        break;
      }
      case Qt::Key_A: {     // accept the matching (creates an edge in the graph, connecting the two maps)
        if (_last_relation) {
          if(_serializable_objects) { _serializable_objects->push_back(_last_relation.get()); } 
          relations.insert(_last_relation);
          _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
        }
        update(); // Refresh display
        break;
      }
      case Qt::Key_C: {     // deselect (and hide) all local maps
        _selected_objects.clear();
        update(); // Refresh display
        break;
      }
      case Qt::Key_V: {     // select (and show) all local maps
        for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
          _selected_objects.insert(it->get());
        }
        update(); // Refresh display
        break;
      }
      case Qt::Key_R: {     // remove edge between two maps
        if (_selected_objects.size() != 2) {
          cerr << "to remove an edge you need to have exactly two objects, I do nothing" << endl;
          return;
        }
        LocalMap* reference, *current;
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        reference = dynamic_cast<LocalMap*>(*it);
        it++;
        current = dynamic_cast<LocalMap*>(*it);
        if (! reference || ! current) {
          cerr << "invalid object types, I do nothing" << endl;
          return;
        }

        for(BinaryNodeRelationSet::iterator it = relations.begin(); it != relations.end(); ++it) {
          const std::tr1::shared_ptr<BinaryNodeRelation>& r = *it;
          if(r->from() == reference && r->to() == current ||
             r->from() == current && r->to() == reference) {        
            relations.erase(it); 
            if(_serializable_objects) { _serializable_objects->remove(it->get()); }                                 
          }
        }
        update(); // Refresh display
        break;
      }
      case Qt::Key_S: {     // save result to disk (requires -o command line option for the filename)
        if(_output_filename == "" || !_serializable_objects) { 
          std::cerr << "[WARNING]: output filename not given in the command line when this was started, I do nothing" << std::endl; 
          return;
        }
        
        Serializer ser;
        ser.setFilePath(_output_filename);
        ser.setBinaryPath(_output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
        for(std::list<Serializable*>::iterator it = _serializable_objects->begin(); 
            it != _serializable_objects->end(); 
            ++it) {
          Serializable* s = *it;
          ser.writeObject(*s);
        }
        break;
      }
      case Qt::Key_T: {
        _reset_trans ^= _reset_trans;   // toggle the state
        std::cerr << "translation reset for matching is now " << (_reset_trans ? "on" : "off") << std::endl;
        break;
      }
      case Qt::Key_Plus: {  // increase voxel size for matching
        if( _voxel_size >= 1.0 ) {
          std::cerr << "maximum voxel size reached (size=" << _voxel_size << ")" << std::endl;
          break;
        }
        _voxel_size *= 2.0;
        std::cerr << "voxel size = " << _voxel_size << std::endl;
        break;
      }
      case Qt::Key_Minus: { // decreases voxel size for matching
        if( _voxel_size <= 0.005 ) {
          std::cerr << "minimum voxel size reached (size=" << _voxel_size << ")" << std::endl;
          break;
        }
        _voxel_size *= 0.5;
        std::cerr << "voxel size = " << _voxel_size << std::endl;
        break;
      }
      case Qt::Key_4: {     // moves the selected cloud to the left (using the num-pad is recommended for this)
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().x() += 0.1;
        current->setTransform( T );
        std::cerr << "move +x" << std::endl;
        update();
        break;
      }
      case Qt::Key_6: {
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().x() -= 0.1;
        current->setTransform( T );
        std::cerr << "move -x" << std::endl;
        update();
        break;
      }
      case Qt::Key_2: {
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().z() += 0.1;
        current->setTransform( T );
        std::cerr << "move +z" << std::endl;
        update();
        break;
      }
      case Qt::Key_8: {
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().z() -= 0.1;
        current->setTransform( T );
        std::cerr << "move -z" << std::endl;
        update();
        break;
      }
      case Qt::Key_9: {
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().y() += 0.1;
        current->setTransform( T );
        std::cerr << "move +y" << std::endl;
        update();
        break;
      }
      case Qt::Key_3: {
        std::set<MapNode*>::iterator it = _selected_objects.begin();
        LocalMap *current;
        
        switch( _selected_objects.size() ) {
          case 1:
            current = dynamic_cast<LocalMap*>(*it);
            break;
          case 2:
            ++it;
            current = dynamic_cast<LocalMap*>(*it);
            break;
          default:
            return;
        }
        Eigen::Isometry3f T = current->transform();
        T.translation().y() -= 0.1;
        current->setTransform( T );
        std::cerr << "move -y" << std::endl;
        update();
        break;
      }
      default:
        QGLViewer::keyPressEvent(e);    
    }
  }    

  std::string _output_filename;
  std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
  std::list<Serializable*>* _serializable_objects;
  double _voxel_size;
  bool _reset_trans;
};

const char* banner[] = {
  "fps_refiner_node: does on line manual loop closure, and publishes the transforms that map the tracker maps into a local map",
  "usage:",
  " fps_refiner_app [options] input_filename",  
  " where: ",
  " -h          [], prints this help",
  " -o    [string], output filename where to write the refined graph",
  "once the gus starts",
  " 1: toggles/untoggles the current view (and saves a lot of bandwidth)",
  " shift-left-click on a node: highlights the local map of the node",
  " M: matches the local maps (if there are only two higlighted)",
  " A: accepts the most recent match (press any other key to discard)",
  " O: optimizes the network",
  " R: remove the edges between the local maps (if there are only two higlighted)",
  " S: save the current graph",
  " +: increase voxel size (used when matching maps)",
  " -: decrease voxel size (used when matching)",
  0
};

int main (int argc, char** argv) {
  int c = 1;
  std::string output_filename = "";
  std::string input_filename = "";              
  while (c<argc) {
    if (! strcmp(argv[c], "-h")) {
      system_utils::printBanner(banner);
      return 0;
    } 
    else if (! strcmp(argv[c], "-o")) {
      c++;
      output_filename = std::string(argv[c]);
    }
    else {
      input_filename = std::string(argv[c]);
    }
    c++;
  }
  std::list<Serializable*> objects;
  Deserializer des;
  des.setFilePath(input_filename);
  Serializable* o;
  QApplication app(argc, argv);
  RefinerViewer viewer(output_filename, &objects);
  while ( (o = des.readObject()) ){
    LocalMap* lmap = dynamic_cast<LocalMap*>(o);
    if (lmap)
      viewer.nodes.addElement(lmap);
    BinaryNodeRelation* rel = dynamic_cast<BinaryNodeRelation*> (o);
    if (rel)
      viewer.relations.insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));

    objects.push_back(o);
  }
  cerr << "Read: " << objects.size() << " elements" << endl;
  cerr << "Read: " << viewer.nodes.size() << " local maps" << endl;
  
  viewer.show();
  app.exec();
  return 0;
}

















