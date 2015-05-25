#include "globals/system_utils.h"
#include "txt_io/message_reader.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/static_transform_tree.h"
#include "txt_io/message_seq_synchronizer.h"
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
  "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
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
  std::vector<std::string> depth_topics;
  std::vector<std::string> rgb_topics;

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
    } else if (! strcmp(argv[c], "-max_distance")){
      c++;
      max_distance = atof(argv[c]);
    } else if (! strcmp(argv[c], "-min_distance")){
      c++;
      min_distance = atof(argv[c]);
    } else if (! strcmp(argv[c], "-t")){
      c++;
      depth_topics.push_back(argv[c]);
    } else if (! strcmp(argv[c], "-rgbt")){
      c++;
      rgb_topics.push_back(argv[c]);
    } else if (! strcmp(argv[c], "-config")){
      c++;
      config = argv[c];
    } else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    } else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    } else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
    } else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    } else if (! strcmp(argv[c], "-o")){
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
  
  
  std::vector<MessageSeqSynchronizer> synchronizers(depth_topics.size());
  if (rgb_topics.size()>0){
    if (rgb_topics.size()!=depth_topics.size()){
      cerr << "fatal error the number of RGB topics should be the same as the -t topics" << endl;
      return 0;
    }
    for (size_t i=0; i<depth_topics.size(); i++){
      std::vector<string> depth_plus_rgb_topic;
      depth_plus_rgb_topic.push_back(depth_topics[i]);
      depth_plus_rgb_topic.push_back(rgb_topics[i]);
      synchronizers[i].setTopics(depth_plus_rgb_topic);
    }
  } else {
    for (size_t i=0; i<depth_topics.size(); i++){
      std::vector<string> depth_topic;
      depth_topic.push_back(depth_topics[i]);
      synchronizers[i].setTopics(depth_topic);
    }
  }

  cerr << "constructing tracker ... ";
  if (depth_topics.size() < 2 || single) {
    tracker = Tracker::makeTracker(alignerType, config);
  } else {
    MultiTracker* multi_tracker = MultiTracker::makeTracker(alignerType, config);
    multi_tracker->init(depth_topics);
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

  cerr << "opening file " << filename << endl;

  cerr << "ALL IN PLACE" << endl;
    
  QApplication* app=new QApplication(argc, argv);
  TrackerViewer* viewer = new TrackerViewer(tracker);
  viewer->show();

  int cloud_count = 0;

  MessageReader reader;
  reader.open(filename);

  bool running = true;
  while (reader.good()) {
    if (running) {
      BaseMessage* msg = reader.readMessage();
      if (! msg)
	continue;
      PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg);
      if (! img)
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


      PinholeImageMessage* depth_img=0, *rgb_img=0;
      size_t i;
      for (i=0; i<synchronizers.size(); i++){
	synchronizers[i].putMessage(img);
	if (synchronizers[i].messagesReady()) {
	  depth_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[0].get());
	  if (synchronizers[i].messages().size()>1)
	    rgb_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[1].get());
	  break;
	}
      }
      if (! depth_img)
	continue;
      RGBImage rgb_image;
      if (rgb_img)
	rgb_image =rgb_img->image();
      tracker->processFrame(depth_img->image(),
			      rgb_image, 
			      depth_img->cameraMatrix(),
			      depth_img->depthScale(),
			      depth_img->seq(),
			      depth_img->timestamp(),
			      depth_img->topic(),
			      depth_img->frameId(),
			      depth_img->offset(),
			      depth_img->odometry(),
			      odom_info);
      
      std::cerr << "T: " << t2v(tracker->globalT()).transpose() << std::endl;
      if (tracker->isTrackBroken()){
	saveCloud(output_filename,cloud_count);
	tracker->clearStatus();
      }
      viewer->updateGL();
      app->processEvents();
      synchronizers[i].reset();
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
      default:;
      }
      viewer->keyEventProcessed();
    }
  }
  saveCloud(output_filename,cloud_count);
}
