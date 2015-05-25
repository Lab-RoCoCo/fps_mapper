#include "globals/system_utils.h"
#include "txt_io/message_reader.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/static_transform_tree.h"
#include "core/depth_utils.h"
#include "tracker/base_triggers.h"
#include "tracker/multi_tracker.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include "tracker_viewers/tracker_viewer.h"
#include "qapplication.h"
#include <qevent.h>


using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;
using namespace system_utils;

Tracker* tracker = 0;

const char* banner[]= {
  "fps_tracker_gui_app: offline tracker working on dump files written with fps_message_dumper_node",
  "usage:",
  " fps_tracker_gui_app [options] <dump filename>",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -o:            [string] output filename where to write the model, default \"\"",
  "once the gui has started you can:",
  " -dump the current cloud (W key)",
  " -pause/start the tracker(P key)",
  " -reset the cloud        (R key)",
  0
};


void saveCloud(const std::string& prefix, int& num ){
  if (!tracker->referenceModel())
    return;
  if (!prefix.length())
    return;

  char buf[1024];
  sprintf(buf, "%s-%05d.cloud", prefix.c_str(), num);
  Cloud c = *tracker->referenceModel();
  c.transformInPlace(tracker->globalT());
  ofstream os(buf);
  c.write(os);
  cerr << "Saving cloud in file " << buf << endl;
  num++;
}

int main(int argc, char **argv) {
  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string transforms_filename = "";
  std::string output_filename="";
  std::vector<std::string> topics;

  float bad_points_ratio = 0.1;
  float damping = 100;
  float tbb = 5;
  float obb = 1;
  int shrink = 1;
  std::string filename = "";
  float min_distance = 0;
  bool cam_only = false;
  int c = 1;
  float max_distance = 3;
  bool single = false;
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-cam_only")){
      cam_only=true;
      cerr << "CAM_ONLY" << endl;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    } else if (! strcmp(argv[c], "-aligner")){
      c++;
      alignerType = argv[c];
    }
    else if (! strcmp(argv[c], "-max_distance")){
      c++;
      max_distance = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-min_distance")){
      c++;
      min_distance = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-t")){
      c++;
      topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-config")){
      c++;
      config = argv[c];
    }
    else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    }
    else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
    } else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    }

    else if (! strcmp(argv[c], "-o")){
      c++;
      output_filename = argv[c];
    } else {
      filename = argv[c];
      break;
    }
    c++;
  }
  if (filename.length()==0){
    cerr << "Error: you have to provide an input filename" << endl;
    return 0;
  } else {
    cerr << "reading from file " << filename << endl;
  }

  StaticTransformTree * _transforms = 0;
  if (transforms_filename.length()){
    _transforms = new StaticTransformTree;
    _transforms->load(transforms_filename);
  }
  
  
  cerr << "constructing tracker ... ";
  if (topics.size() < 2 || single) {
    tracker = Tracker::makeTracker(alignerType, config);
  } else {
    MultiTracker* multi_tracker = MultiTracker::makeTracker(alignerType, config);
    multi_tracker->init(topics);
    tracker = multi_tracker;
  }
  if (! tracker) {
    cerr << "unknown tracker type [" << alignerType << "] aborting" << endl;
    return 0;
  }
  tracker->setBadPointsRatio(bad_points_ratio);
  tracker->aligner().solver().setDamping(damping);
  tracker->setImageShrink(shrink);
  tracker->setMaxDistance(max_distance);
  tracker->setMinDistance(min_distance);

  cerr << " Done" << endl;


  // new VerboseTrigger(tracker, Tracker::TRACK_BROKEN, 0, "TRACK BROKEN!!!");

  //new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
  //		     "frame_count <frame_count> <seq> Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  cerr << "ALL IN PLACE" << endl;
    
  QApplication* app=new QApplication(argc, argv);
  TrackerViewer* viewer = new TrackerViewer(tracker);
  viewer->resize(640,512);
  viewer->show();
  viewer->setFollowCamera(true);
  glPointSize(3);
  int cloud_count = 0;

  MessageReader reader;
  reader.open(filename);

  bool running = true;

  PinholeImageMessage* depth_img = 0;
  PinholeImageMessage* rgb_img = 0;
  PinholeImageMessage* img;
  bool has_rgb = true;

  ofstream os ("assembled_pics-depth.txt");
  std::string last_external_image_filename;
  while (reader.good()) {
    if (running) {
      BaseMessage* msg = reader.readMessage();
      if (! msg)
	continue;
      
      img = dynamic_cast<PinholeImageMessage*>(msg);
      if (! img)
	continue;
      if (img->image().type()==CV_8UC3) {
	if (rgb_img) {
	  delete rgb_img;
	}
	rgb_img = img;
      }
      if (img->image().type()==CV_16UC1) {
	if (depth_img)
	  delete depth_img;
	depth_img = img;
      }
      
      if (img->topic() == "/camera1/rgb/image_raw"){
	last_external_image_filename = img->binaryFullFilename();
      }
      if (! last_external_image_filename.length())
	continue;

      if (! depth_img || 
	  (! rgb_img ||
		     depth_img->seq()!=rgb_img->seq()))
	continue;

      Matrix6f odom_info;
      odom_info.setIdentity();
      if (! img->hasOdom()){
	odom_info.setZero();
      } 
    

      if (_transforms) 
	_transforms->applyTransform(*img);

      if(cam_only)
	img->setOffset(Eigen::Isometry3f::Identity());
      if(cam_only)
	img->setOdometry(Eigen::Isometry3f::Identity());
      if (has_rgb) {
	tracker->processFrame(depth_img->image(),
			      rgb_img->image(),
			      depth_img->cameraMatrix(),
			      depth_img->depthScale(),
			      depth_img->seq(),
			      depth_img->timestamp(),
			      depth_img->topic(),
			      depth_img->frameId(),
			      depth_img->offset(),
			      depth_img->odometry(),
			      odom_info);
      } else {
	tracker->processFrame(depth_img->image(),
			      RGBImage(),
			      depth_img->cameraMatrix(),
			      depth_img->depthScale(),
			      depth_img->seq(),
			      depth_img->timestamp(),
			      depth_img->topic(),
			      depth_img->frameId(),
			      depth_img->offset(),
			      depth_img->odometry(),
			      odom_info);
      }
      std::cerr << "T: " << t2v(tracker->globalT()).transpose() << std::endl;
      if (tracker->isTrackBroken()){
	saveCloud(output_filename,cloud_count);
	tracker->clearStatus();
      }
      viewer->updateGL();
      char cloud_name[1024];
      
      sprintf(cloud_name, "cloud-depth%05d.jpg", depth_img->seq());
      viewer->saveSnapshot(QString(cloud_name), true);
      os  << last_external_image_filename << " " ;
      os  << cloud_name << " ";
      os  << depth_img->binaryFullFilename();
      os  << " " << rgb_img->binaryFullFilename();
      os << endl;

      app->processEvents();
      //delete img;
    } else {
      app->processEvents();
      usleep(20000);
    }
    QKeyEvent* event=viewer->lastKeyEvent();
    if (event){
      switch(event->key()) {
      case Qt::Key_W: 
	saveCloud(output_filename,cloud_count);
	break;
      case Qt::Key_R: 
	tracker->clearStatus();
	break;
      case Qt::Key_P: 
	running = ! running;
	break;
      case Qt::Key_F: 
	viewer->setFollowCamera(!viewer->followCamera());
	break;
      default:;
      }
      viewer->keyEventProcessed();
    }
  }
  saveCloud(output_filename,cloud_count);
}
