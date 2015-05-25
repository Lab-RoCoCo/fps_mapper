#include "globals/system_utils.h"
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <iostream>
#include "txt_io/static_transform_tree.h"
#include "core/depth_utils.h"
#include "core/nn_aligner.h"
#include "core/projective_aligner.h"
#include <fstream>
#include "tracker/tracker.h"
#include "tracker/multi_tracker.h"
#include "tracker/base_triggers.h"
#include "cloud_publisher_trigger.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "ros_wrappers/image_message_listener.h"
#include "tracker/call_tracker_trigger.h"

#include "tracker_viewers/tracker_viewer.h"
#include "qapplication.h"
#include <qevent.h>


using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;


tf::TransformListener * listener = 0;
std::string base_link_frame_id = "";
std::string odom_frame_id = "/odom"; 
Tracker* tracker = 0;


const char* banner[]= {
  "fps_tracker_node: offline tracker working as ros node",
  "usage:",
  " fps_tracker_node [options]",
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
  "  -base_link_frame_id [string]: if specified listens for odom, and tracks the pose of the base_link specified",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -gui:          [flag]  enables the gui for fancy visualization"
  "  -o:            [string] output filename where to write the model, default \"\"",
  "once the gui has started you can ump the current cloud by pressing W ",
  0
};

void saveCloud(const std::string& prefix, int& num ){
  if (!tracker->referenceModel())
    return;
  if (!prefix.length())
    return;

  char buf[1024];
  sprintf(buf, "%s-%05d.cloud", prefix.c_str(), num);
  ofstream os(buf);
  tracker->referenceModel()->write(os);
  cerr << "Saving cloud in file " << buf << endl;
  num++;
}


int main(int argc, char **argv) {
  std::vector<string> topics;
  std::list<ImageMessageListener*> camera_listeners;

  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string transforms_filename = "";
  std::string output_filename="";

  float bad_points_ratio = 0.1;
  float damping = 100;
  float tbb = 5;
  float obb = 1;
  int shrink = 1;
  float min_distance = 0;
  bool cam_only = false;
  int c = 1;
  float max_distance = 3;
  bool gui = false;
  bool single = false;
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      system_utils::printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-cam_only")){
      cam_only=true;
      cerr << "CAM_ONLY" << endl;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    } else if (! strcmp(argv[c], "-gui")){
      gui=true;
      cerr << "enabled gui" << endl;
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
    } else if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id = argv[c];
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
    } 
    c++;
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


  new VerboseTrigger(tracker, Tracker::TRACK_BROKEN, 0, "TRACK BROKEN!!!");

  new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
		     "frame_count <frame_count> <seq> Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  cerr << "ALL IN PLACE" << endl;
 
  ros::init(argc, argv, "fps_tracker_node");
  if (base_link_frame_id.length()>0){
    cerr << "making listener" << endl;
    listener = new tf::TransformListener(ros::Duration(60.0));
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);

  QApplication* app = 0;
  TrackerViewer* viewer = 0;
  if (gui) {
    app=new QApplication(argc, argv);
    viewer = new TrackerViewer(tracker);
    viewer->show();
  }

  SensorMessageSorter sorter;
  sorter.setTimeWindow(0.);
  CallTrackerTrigger* caller = new CallTrackerTrigger(&sorter, 0, tracker);
  for (std::vector<std::string>::iterator it = topics.begin(); it!=topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, &sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->subscribe(topic);
    cerr << "subscribing for topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }

  tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;

  fps_mapper::CloudPublisherTrigger* cloud_publisher = 
    new fps_mapper::CloudPublisherTrigger(tracker, 
  					  Tracker::PROCESSING_DONE,
  					  100, nh, broadcaster);

  if (gui) {
    while (ros::ok()){
      ros::spinOnce();
      QKeyEvent* event=viewer->lastKeyEvent();
      if (event){
	switch(event->key()) {
	case Qt::Key_R: 
	  tracker->clearStatus();
	  break;
	default:;
	}
	viewer->keyEventProcessed();
      }
      viewer->updateGL();
      app->processEvents();
    }
  } else {
    ros::spin();
  }
}

