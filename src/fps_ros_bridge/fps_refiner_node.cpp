#include <fstream>
#include "globals/system_utils.h"
#include "global_optimization/g2o_bridge.h"
#include "core/cloud.h"
#include "core/nn_aligner.h"
#include "fps_map/image_map_node.h"
#include "fps_map/local_map.h"
#include "fps_map_viewers/trajectory_viewer.h"
#include "qapplication.h"
#include "qevent.h"
#include <stdexcept>
#include "boss/deserializer.h"
#include "boss/trusted_loaders.h"
#include "fps_ros_msgs.h"
#include "fps_mapper/StampedCloudMsg.h"
#include "cloud_publisher_trigger.h"
#include "gl_helpers/simple_viewer.h"
#include "gl_helpers/opengl_primitives.h"
#include "local_map_listener.h"
#include "local_map_viewer.h"
#include "fps_mapper/MapUpdateMsg.h"

using namespace fps_mapper;
using namespace boss;
using namespace std;
using namespace fps_mapper;

BaseCameraInfo cinfo;
ImageMapNode tnode;
LocalMap lmap;
NNAligner aligner;
BinaryNodeRelation rel;

BinaryNodeRelation* matchLocalMaps(LocalMap& reference, LocalMap& current){
  cerr << "Matching" << endl;
  // step 1, determine the transformation between reference and current
  Eigen::Isometry3f T = reference.transform().inverse() * current.transform();
  //T.translation().setZero();
  // step 2 prepare the aligner

  Cloud tempRef(*reference.cloud());
  Cloud tempCurr(*current.cloud());
  voxelize(tempRef, 0.05);
  voxelize(tempCurr, 0.05);
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

class RefinerViewer: public LocalMapViewer{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RefinerViewer(std::string output_filename_ = "") {
    _output_filename = output_filename_; 
    _last_local_map = 0;
    _last_update_seq = 0;
    _last_local_map_pose.setIdentity();
    _delta_pose.setIdentity();
  }
  virtual void onNewLocalMap(LocalMap* lmap) {
    _serializable_objects.push_back(lmap);
     _last_local_map_pose = lmap->transform();
    _last_local_map = lmap;
    LocalMapViewer::onNewLocalMap(lmap);
  }

  virtual void onNewRelation(BinaryNodeRelation* r) {
    _serializable_objects.push_back(r);
    LocalMapViewer::onNewRelation(r);
    r->to()->setTransform(r->from()->transform()*r->transform());
    
    publishUpdates();
  }

  virtual void onNewCameraInfo(BaseCameraInfo* cam) {
    _serializable_objects.push_back(cam);		
    LocalMapViewer::onNewCameraInfo(cam);
  }
  
  virtual void onNewNode(MapNode* n) {
    _serializable_objects.push_back(n);
    if (_last_local_map){
      _delta_pose = _last_local_map->transform()*_last_local_map_pose.inverse();
      if (_tf_broadcaster) {
	char buf [1024];
	_tf_broadcaster->sendTransform(tf::StampedTransform(eigen2tfTransform(_delta_pose), 
							    ros::Time(n->timestamp()), 
							    "global_map_origin_frame_id", 
							    "tracker_origin_frame_id"));
      }
    } 
    LocalMapViewer::onNewNode(n);
 }


  virtual void draw() {
    // determine the offset between the last local map (according to the tracker)
    // and the actual position of the last local map

    glPushMatrix();
    GLHelpers::glMultMatrix(_delta_pose);

    
    glPushMatrix();
    GLHelpers::glMultMatrix(_ref_pose);
    glColor3f(0.5, 0.5, 0.5);
    if (_show_clouds)
      _ref_cloud.draw();
    glPopMatrix();

    glPushMatrix();
    GLHelpers::glMultMatrix(_curr_pose);
    glPushMatrix();
    glColor3f(0.2, 0.2, 1.0);
    if (_show_clouds)
      _curr_cloud.draw();
    glScalef(0.2, 0.2, 0.2);
    GLHelpers::drawReferenceSystem();
    glPopMatrix();
    glPopMatrix();

    for (std::list<MapNode*>::iterator it = _temp_nodes.begin(); it!=_temp_nodes.end(); it++){
      (*it)->draw();
    }
    glPopMatrix();

    TrajectoryViewer::draw();
    _need_redraw = false;
  }

  void init(ros::NodeHandle& n, tf::TransformListener* listener, tf::TransformBroadcaster* broadcaster = 0){
    _tf_broadcaster = broadcaster;
    LocalMapViewer::init(n,listener);
    _updates_pub=n.advertise<fps_mapper::MapUpdateMsg>("/global_optimizer/map_updates",100);
  }

  void keyPressEvent(QKeyEvent *e)
  {    
    // Defines the Alt+R shortcut.
    if ((e->key() == Qt::Key_M))
      {
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
	_last_relation = std::tr1::shared_ptr<BinaryNodeRelation>(matchLocalMaps(*reference, *current));
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_O))
      {
	bridge.psToG2o(relations, nodes);
	bridge.optimize();
	bridge.g2oToPs(nodes);
	update(); // Refresh display
	publishUpdates();
      }
    else if ((e->key() == Qt::Key_A))
      {
	if (_last_relation) {
	  _serializable_objects.push_back(_last_relation.get());		
	  relations.insert(_last_relation);	  
	  _last_relation = std::tr1::shared_ptr<BinaryNodeRelation>();
	}
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_C))
      {
	_selected_objects.clear();
	update(); // Refresh display
      }
    else if ((e->key() == Qt::Key_V))
      {
	for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
	  _selected_objects.insert(it->get());
	}
	update(); // Refresh display
      }
    // Remove edge
    else if ((e->key() == Qt::Key_R))
      {
	if (_selected_objects.size() != 2) {
	  cerr << "to an edge you need to have exactly two objects, I do nothing" << endl;
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
	    _serializable_objects.remove(it->get());		
	  }
	}

	update(); // Refresh display
      }
    // Save current graph
    else if ((e->key() == Qt::Key_S))
      {
	if(_output_filename == "") { 
	  std::cerr << "[WARNING]: output filename not given in the command line when this was started, I do nothing" << std::endl; 
	  return;
	}
	
	Serializer ser;
	ser.setFilePath(_output_filename);
	ser.setBinaryPath(_output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
	for(std::list<Serializable*>::iterator it = _serializable_objects.begin(); 
	    it != _serializable_objects.end(); 
	    ++it) {
	  Serializable* s = *it;
	  ser.writeObject(*s);
	}
      }
    else
      LocalMapViewer::keyPressEvent(e);
  }

  void publishUpdates(){
    _last_update_seq++;
    MapUpdateMsg msg;
    msg.updates.resize(nodes.size());
    int k = 0;
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++){
      MapNode *n = it->get();
      msg.updates[k].node_id = n->getId();
      msg.updates[k].transform = eigen2pose(n->transform());
      k++;
    }
    _updates_pub.publish(msg);
  }

  int _last_update_seq;
  std::string _output_filename;
  std::list<Serializable*> _serializable_objects;
  std::tr1::shared_ptr<BinaryNodeRelation> _last_relation;
  LocalMap* _last_local_map;
  Eigen::Isometry3f _last_local_map_pose, _delta_pose;
  tf::TransformBroadcaster* _tf_broadcaster;
  ros::Publisher _updates_pub;
};


const char* banner[] = {
  "fps_refiner_node: does on line manual loop closure, and publishes the transforms that map the tracker maps into a local map",
  "usage:",
  " fps_refiner_node [options]",  
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
  0
};
  

int main(int argc, char** argv) {
  int c = 1;
  std::string output_filename = "";
  while (c<argc) {
    if (! strcmp(argv[c], "-h")) {
      system_utils::printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-o")) {
      c++;
      output_filename = std::string(argv[c]);
    }
    c++;
  }

  ros::init(argc, argv, "refiner");
  ros::NodeHandle n;
  tf::TransformListener* listener = new tf::TransformListener(ros::Duration(60.0));
  tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;
  QApplication* app=0; 
  RefinerViewer* viewer=0;
  
  app=new QApplication(argc, argv);
  viewer = new RefinerViewer(output_filename);
  viewer->show();
  viewer->init(n, listener, broadcaster);

  while(ros::ok()){
    ros::spinOnce();
    app->processEvents();
    if (viewer->needRedraw()) {			
      viewer->updateGL();
    }
  }
}
