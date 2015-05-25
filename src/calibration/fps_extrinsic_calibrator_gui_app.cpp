#include "globals/system_utils.h"
#include "txt_io/message_reader.h"
#include "txt_io/message_writer.h"
#include "txt_io/static_transform_message.h"
#include "txt_io/static_transform_tree.h"

#include "txt_io/pinhole_image_message.h"
#include "core/depth_utils.h"
#include "tracker/base_triggers.h"
#include "tracker_viewers/tracker_viewer.h"
#include "tracker/multi_tracker.h"

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include "qglviewer.h"
#include "qapplication.h"

#include "calibration_measurement.h"
#include "calibration_prior.h"
#include "camera_calibration_solver.h"
#include "extrinsic_calibrator_trigger.h"

using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;



Tracker* tracker = 0;

const char* banner[]= {
  "fps_extrinsic_calibrator_gui_app: tool for calibrating the pose of the camera w.r.t.the base",
  "usage:",
  " fps_extrinsic_calibrator_gui_app [options] <dump filename>",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -o:            [string] output prefix of the intermediate calibration steps, default \"\"",
  0
};



int main(int argc, char **argv) {
  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string output_filename="";
  std::string transforms_filename="";
  float bad_points_ratio = 0.1;
  float damping = 1;
  float tbb = 5;
  float obb = 1;
  int shrink = 1;
  float max_distance = 3;
  float min_distance = 0;
  std::string filename = "";
  std::vector<std::string> topics;
  bool single = false;

  int c = 1;
  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      system_utils::printBanner(banner);
      return 0;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    } 
    else if (! strcmp(argv[c], "-aligner")){
      c++;
      alignerType = argv[c];
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
    else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    }
    else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-max_distance")){
      c++;
      max_distance = atof(argv[c]);
    } else if (! strcmp(argv[c], "-min_distance")){
      c++;
      min_distance = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
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
    system_utils::printBanner(banner);
    cerr << "Error: you have to provide an input filename" << endl;
    return 0;
  } else {
    cerr << "reading from file " << filename << endl;
  }
  
  StaticTransformTree* _transforms = 0;

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
  if (! tracker) {
    cerr << "unknown tracker type [" << alignerType << "] aborting" << endl;
    return 0;
  }
  tracker->setBadPointsRatio(bad_points_ratio);
  tracker->aligner().solver().setDamping(damping);
  tracker->setImageShrink(shrink);
  tracker->setMaxDistance(max_distance);
  tracker->setMinDistance(min_distance);
  cerr << "constructing tracker ... ";

  cerr << " Done" << endl;

  new VerboseTrigger(tracker, Tracker::TRACK_BROKEN, 0, "TRACK BROKEN!!!");

  ExtrinsicCalibratorTrigger* calibrator = new ExtrinsicCalibratorTrigger(tracker, 10);
  cerr << "ALL IN PLACE" << endl;


  if (output_filename.length()) {
    calibrator->setTfFilePrefix(output_filename);
  }
  QApplication* app=new QApplication(argc, argv);
  TrackerViewer* viewer = new TrackerViewer(tracker);
  viewer->show();

  int cloud_count = 0;

  MessageReader reader;
  reader.open(filename);
  cerr << "opening: " << filename << endl;
  while (reader.good()) {
    BaseMessage* msg = reader.readMessage();
    if (! msg)
      continue;
    PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg);
    if (! img)
      continue;
    if (! img->hasOdom()){
      throw std::runtime_error("I cannot work without odometry");
    } 


    if (_transforms) {
      _transforms->applyTransform(*img);
    }

    Matrix6f odom_info;
    odom_info.setZero();
    tracker->processFrame(img->image(),
			  RGBImage(),
			  img->cameraMatrix(),
			  img->depthScale(),
			  img->seq(),
			  img->timestamp(),
			  img->topic(),
			  img->frameId(),
			  img->offset(),
			  img->odometry(),
			  odom_info);
    delete msg;
    if (tracker->isTrackBroken()){
      tracker->clearStatus();
    }
    viewer->updateGL();
    app->processEvents();
  }
}
