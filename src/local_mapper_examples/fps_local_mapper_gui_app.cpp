#include <qapplication.h>
#include "txt_io/message_reader.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/static_transform_tree.h"
#include "txt_io/message_seq_synchronizer.h"

#include "core/depth_utils.h"
#include "core/projective_aligner.h"
#include "tracker/tracker.h"
#include "tracker/multi_tracker.h"
#include "tracker/base_triggers.h"
#include "local_mapper/local_map_triggers.h"
#include "local_mapper_viewers/local_mapper_viewer.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>



using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;

Tracker* tracker = 0;

const char* banner[]= {
  "fps_local_mapper_gui_app: offline local mapper working on dump files written with fps_message_dumper_node",
  "usage:",
  " fps_local_mapper_gui_app [options] <dump filename>",
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
  "  -skip :        [int]   skip each x frames, default: 1",
  "  -tbb:          [float] when to break the local map (translation), default: 5",
  "  -obb:          [float] when to break the local map (orientation), default: 1",
  "  -odom_weight   [float] weighting of the odometry (against the tracker), default: 1.0",
  "  -o:            [string] output filename where to write the local maps, default \"\"",
  "once the gui has started, with shift + left-click on a node you toggle the display of the local map",
  0
};

void printBanner(){
  const char** b = banner;
  while(*b){
    cout << *b << endl;
    b++;
  }
}


int main(int argc, char **argv) {
  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string output_filename="";
  bool cam_only=false;
  float bad_points_ratio = 0.1;
  float damping = 100;
  float tbb = 5;
  float obb = 1;
  float odom_weight = 1.0;
  int shrink = 1;
  int skip = 0;
  std::string filename = "";
  std::string transforms_filename = "";
  float max_distance = 3;
  float min_distance = 0;
  int c = 1;
  bool single = false;
  std::vector<std::string> depth_topics;
  std::vector<std::string> rgb_topics;

  while (c<argc){
    if (! strcmp(argv[c], "-h")){
      printBanner();
      return 0;
    } else if (! strcmp(argv[c], "-cam_only")){
      cam_only=true;
      cerr << "CAM_ONLY" << endl;
    } else if (! strcmp(argv[c], "-single")){
      single=true;
      cerr << "single tracker" << endl;
    }
    else if (! strcmp(argv[c], "-aligner")){
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
    else if (! strcmp(argv[c], "-config")){
      c++;
      config = argv[c];
    }
    else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    } 
    else if (! strcmp(argv[c], "-t")){
      c++;
      depth_topics.push_back(argv[c]);
    } else if (! strcmp(argv[c], "-rgbt")){
      c++;
      rgb_topics.push_back(argv[c]);
    } else if (! strcmp(argv[c], "-bpr")){
      c++;
      bad_points_ratio = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-damping")){
      c++;
      damping = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-tf")){
      c++;
      transforms_filename = argv[c];
    }
    else if (! strcmp(argv[c], "-tbb")){
      c++;
      tbb = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-obb")){
      c++;
      obb = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-odom_weight")){
      c++;
      odom_weight = atof(argv[c]);
    }
    else if (! strcmp(argv[c], "-skip")){
      c++;
      skip = atoi(argv[c]);
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
    printBanner();
    cerr << "Error: you have to provide an input filename" << endl;
    return 0;
  } else {
    cerr << "reading from file " << filename << endl;
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
  ProjectiveAligner* aligner = dynamic_cast<ProjectiveAligner*>(&tracker->aligner());
  aligner->finder().setPointsDistance(1);
  aligner->finder().setNormalAngle(1.);
  
  tracker->setImageShrink(shrink);
  tracker->setMaxDistance(max_distance);
  tracker->setMinDistance(min_distance);
  tracker->setFrameSkip(skip);

  cerr << " Done" << endl;


  StaticTransformTree * _transforms = 0;
  if (transforms_filename.length()){
    _transforms = new StaticTransformTree;
    _transforms->load(transforms_filename);
  }

  // trigger examples. 
  // ALWAYS Use the heap as if you allocate the triggers on the stack they are optimized out

  // here we set the following behavior for the tracker:
  // each time the local map becomes big,
  // it has to save a local map and then to reset
  // thus we create two triggers that respond to the event
  // Tracker::TRAJECTORY_BOUNDARIES_REACHED
  // the first (higher priority) makes and saves the local map
  boss::Serializer* ser = 0;
  if (output_filename != "") {
    ser  = new boss::Serializer();
    ser->setFilePath(output_filename);
    ser->setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
  }

  
  LocalMapTrigger* local_map_maker = new LocalMapTrigger(tracker, 
                                                         Tracker::TRACK_GOOD|
                                                         Tracker::TRACK_BROKEN|
                                                         Tracker::REFERENCE_FRAME_RESET|
                                                         Tracker::TRACKING_DONE,
                                                         1, ser);

  local_map_maker->setTrajectoryMaxTranslation(tbb);
  local_map_maker->setTrajectoryMaxOrientation(obb);
  
  //new ClearStatusTrigger(tracker, Tracker::TRACK_BROKEN, 2);

  new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
                     "Frame: <frame_count>, Seq : <seq>, Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  cerr << "ALL IN PLACE" << endl;

  
  QApplication* app=0; 
  LocalMapperViewer* viewer=0;
  app=new QApplication(argc, argv);
  viewer = new LocalMapperViewer(local_map_maker);
  local_map_maker->setLocalMaps(&viewer->nodes);
  local_map_maker->setLocalMapsRelations(&viewer->relations);
  viewer->show();
  

  MessageReader reader;
  reader.open(filename);
  
  cerr << "opened " << filename << endl;

  bool running = true;
  while (reader.good()) {
    
    QKeyEvent* e = viewer->lastKeyEvent();
    if (e && e->key() == Qt::Key_P){
      viewer->keyEventProcessed();
      running = ! running;
    }
  
    if (running) {
      BaseMessage* msg = reader.readMessage();
      if (! msg)
        continue;
      PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg);
      if (! img)
        continue;

      Matrix6f odom_info;
      odom_info.setIdentity();
      odom_info *= odom_weight;
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
  
      viewer->updateGL();
      app->processEvents();
      synchronizers[i].reset();
    } else {
      app->processEvents();
      usleep(20000);
    }
  }

}

