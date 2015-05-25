#include <iostream>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

#include "globals/defs.h"
#include "globals/system_utils.h"

#include "txt_io/message_reader.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/static_transform_tree.h"

#include "fps_kinfu_tracker.h"
#include "kinfu_tracker_viewer.h"

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace pcl::gpu::kinfuLS;

using namespace txt_io;
using namespace fps_mapper;

const char* banner[] = {
  "fps_kinfu_tracker_gui_app: offline kinfu tracker working on dump files written with fps_message_dumper_node",
  "usage:",
  " fps_kinfu_tracker_app [options] <dump filename>",
  " where: ",
  "  -cam_only      [bool]   flag, if set ignores the odometry and operates in the camera frame",
  "  -t             [string] specifies which image topic to use, if unset will use /cemar0/depth/image_raw",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -shrink        [int]    image downscaling (2 means half size), default: 1",
  "  -o             [string] output filename where to write the model, default \"\"",
  "  -vs            [float]  define integration volume size in meters",
  "  -sd            [float]  define shifting threshold (distance target-point / cube center) in meters",
  "  -rows          [int]    number of rows of each depth image",
  "  -cols          [int]    number of columns of each depth image",
  "  -fx            [float]  focal length x, default is 262.5",
  "  -fy            [float]  focal length y, defualt is 262.5",
  "  -cx            [float]  principal point x, default is 159.75f",
  "  -cy            [float]  principal point y, default is 119.75f",
  " once the gui has started you can:",
  "  -dump the current cloud  (W key)",
  "  -pause/start the tracker (P key)",
  "  -reset the cloud         (R key)",
  0
};

int main(int argc, char** argv) {
  // Input handling
  std::string transforms_filename = "";
  std::string output_filename = "";
  std::string topic = "/cemar0/depth/image_raw";

  int rows = 240;
  int cols = 320;
  Eigen::Matrix3f K;
  K <<  
    262.5f,   0.0f, 159.75f, 
      0.0f, 262.5f, 119.75f, 
      0.0f,   0.0f,    1.0f;
  Eigen::Vector3f volume_size = Vector3f::Constant(pcl::device::kinfuLS::VOLUME_SIZE);
  float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
  float bad_points_ratio = 0.1f;
  int shrink = 1;
  std::string filename = "";
  bool cam_only = false;
  int c = 1;
  while( c < argc) {
    if(!strcmp(argv[c], "-h")) {
      system_utils::printBanner(banner);
      return 0;
    } 
    else if(!strcmp(argv[c], "-cam_only")) { cam_only = true; } 
    else if(!strcmp(argv[c], "-t")) {
      c++;
      topic = argv[c];
    }
    else if(!strcmp(argv[c], "-shrink")) {
      c++;
      shrink = atoi(argv[c]);
    }
    else if(!strcmp(argv[c], "-tf")) {
      c++;
      transforms_filename = argv[c];
    }
    else if(!strcmp(argv[c], "-o")) {
      c++;
      output_filename = argv[c];
    } 
    else if(!strcmp(argv[c], "-vs")) {
      c++;
      volume_size = Vector3f::Constant(atof(argv[c]));
    }
    else if(!strcmp(argv[c], "-sd")) {
      c++;
      shift_distance = atof(argv[c]);
    } 
    else if(!strcmp(argv[c], "-rows")) {
      c++;
      rows = atoi(argv[c]);
    } 
    else if(!strcmp(argv[c], "-cols")) {
      c++;
      cols = atoi(argv[c]);
    } 
    else if(!strcmp(argv[c], "-fx")) {
      c++;
      K(0, 0) = atof(argv[c]);
    } 
    else if(!strcmp(argv[c], "-fy")) {
      c++;
      K(1, 1) = atof(argv[c]);
    } 
    else if(!strcmp(argv[c], "-cx")) {
      c++;
      K(0, 2) = atof(argv[c]);
    } 
    else if(!strcmp(argv[c], "-cy")) {
      c++;
      K(1, 2) = atof(argv[c]);
    } 
    else {
      filename = argv[c];
      break;
    }
    c++;
  }

  if(filename.length() == 0) {
    std::cout << "[ERROR]: you have to provide an input filename" << endl;
    return 0;
  } 
  else { std::cout << "[INFO]: reading from file " << filename << std::endl; }

  StaticTransformTree * _transforms = 0;
  if(transforms_filename.length()) {
    _transforms = new StaticTransformTree;
    _transforms->load(transforms_filename);
  }
  
  std::cout << "[INFO]: constructing kinfu tracker...";  
  FPSKinfuTracker* tracker = new FPSKinfuTracker(volume_size, shift_distance, rows / shrink, cols / shrink);
  tracker->setDepthIntrinsics(K(0, 0) / (float)shrink, K(1, 1) / (float)shrink, 
			      K(0, 2) / (float)shrink, K(1, 2) / (float)shrink);
  std::cout << " done" << std::endl;

  int cloud_count = 0;

  MessageReader reader;
  reader.open(filename);

  std::cout << "[INFO]: constructing kinfu tracker viewer...";  
  QApplication* app = new QApplication(argc, argv);
  KinfuTrackerViewer* viewer = new KinfuTrackerViewer(tracker);
  viewer->show();
  std::cout << " done" << std::endl;

  bool running = true;
  while(reader.good()) {
    if(running && !tracker->icpIsLost()) {
      BaseMessage* msg = reader.readMessage();
      if(!msg) { continue; }
      PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg);
      if(!img) { continue; }
      if(strcmp(topic.c_str(), img->topic().c_str())) { continue; }
      if(_transforms) { _transforms->applyTransform(*img); }
      std::cout << "[INFO]: processing " << img << std::endl;
      if(cam_only) { 
      	img->setOffset(Eigen::Isometry3f::Identity());
      	img->setOdometry(Eigen::Isometry3f::Identity());
      }
     
      tracker->processFrame(img->image(), img->cameraMatrix(), 
      			    img->depthScale(), shrink, 
      			    img->topic(), img->frameId(),
      			    img->offset(), img->odometry());

      viewer->updateGL();
      app->processEvents();
      delete img;
    }
    else {
      app->processEvents();
      usleep(20000);
    }
    
    QKeyEvent* event = viewer->lastKeyEvent();
    if(event) {
      switch(event->key()) {
      case Qt::Key_P: 
	running = !running;
	break;
      default:;
      }
      viewer->keyEventProcessed();
    }
  }

  return 0;
}
