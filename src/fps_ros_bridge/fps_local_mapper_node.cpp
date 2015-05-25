#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <iostream>
#include "core/depth_utils.h"
#include "core/nn_aligner.h"
#include "core/projective_aligner.h"
#include <fstream>
#include "qglviewer.h"
#include "qapplication.h"
#include "tracker/tracker.h"
#include "tracker/base_triggers.h"
#include "local_mapper/local_map_triggers.h"
#include "tracker/multi_tracker.cpp"
#include "txt_io/tf_overrider_trigger.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "globals/system_utils.h"
#include "ros_wrappers/image_message_listener.h"
#include "txt_io/message_enlister_trigger.h"
#include "txt_io/message_dumper_trigger.h"
#include "txt_io/pinhole_image_message.h"
#include "txt_io/message_seq_synchronizer.h"

#include "fps_ros_msgs.h"
#include "local_mapper_ros.h"
#include <pthread.h>
#include "cloud_publisher_trigger.h"
#include <stdexcept>
using namespace std;
using namespace txt_io;
using namespace fps_mapper;
using namespace Eigen;

Tracker* tracker = 0;
VerboseTrigger* vt = 0;

SensorMessageSorter* sorter = new SensorMessageSorter;
SensorMessageList* _messages = new SensorMessageList;
bool thread_run = false;
MessageWriter* writer = 0;
ProfilerTrigger* profiler = 0;
std::vector<MessageSeqSynchronizer> synchronizers;
float odom_weight = 1.0;		// odomerty weight, default is 1.0


const char* banner[]= {
  "fps_local_mapper_gui_app: offline local mapper working on dump files written with fps_message_dumper_node",
  "usage:",
  " fps_local_mapper_gui_app [options]",
  " where: ",
  "  -aligner:      [string] aligner type [projective or nn], default: projective",
  "  -config:       [string] config type [Xtion640x480, Kinect640x480, Xtion320x240], default: Xtion320x240",
  "  -max_distance: [float] max range of the beams to consider for alignment, default 3",
  "  -min_distance: [float] min range of the beams to consider for alignment, default 0",
  "  -cam_only:     flag, if set ignores the odometry and operates in the camera frame",
  "  -t:            [string] specifies which image topic to use, if unset will use all",
  "                          to issye multiple topics use \"-t <topic1>  -t <topic2> .. -t <topicN> \"",  "  -rgbt:         [string] specifies which rgb image topics to use. same as above. The number of -rgbt should match the order and the number of -t.",
  "  -single        [flag] if set uses the single projector, otherwise it uses the multi projector on all topics",
  "  -tf            [string] if set overrides the transforms in the file with theones of the file provided",
  "  -base_link_frame_id [string]: if specified listens for odom, and tracks the pose of the base_link specified",
  "  -bpr:          [float] bad points ratio [float], default: 0.1",
  "  -damping:      [float] solver damping, default: 100",
  "  -shrink :      [int]   image downscaling (2 means half size), default: 1",
  "  -skip :        [int]   skip each x frames, default: 1",
  "  -tbb:          [float] when to break the local map (translation), default: 3",
  "  -obb:          [float] when to break the local map (orientation), default: 8",
  "  -odom_weight:  [float] odometry weighting, default: 1.0",
  "  -o:            [string] output filename where to write the local maps, default \"\"",
  "once the gui has started, with shift + left-click on a node you toggle the display of the local map",
  0
};


void* run_local_mapper(void*) {
  while(thread_run) {
    if (! _messages || _messages->empty()){
      usleep(100000);
      continue;
    }

    std::tr1::shared_ptr<BaseSensorMessage> msg = _messages->front();
    _messages->pop_front();
    PinholeImageMessage* img = dynamic_cast<PinholeImageMessage*>(msg.get());
    if (! img)
      return 0 ;

    cerr << "Got  stuff" << endl;

    Matrix6f odom_info;
    odom_info.setIdentity();
    odom_info *= odom_weight;
    if (! img->hasOdom()){
      odom_info.setZero();
    } 
    
    if  (writer) {
      writer->writeMessage(*img);
      img->writeBack();
    } else
      img->untaint();


    PinholeImageMessage* depth_img=0, *rgb_img=0;
    size_t i;
    for (i=0; i<synchronizers.size(); i++){
      synchronizers[i].putMessage(msg);
      if (synchronizers[i].messagesReady()) {
	depth_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[0].get());
	if (synchronizers[i].messages().size()>1)
	  rgb_img=dynamic_cast<PinholeImageMessage*>(synchronizers[i].messages()[1].get());
	break;
      }
    }
    if (! depth_img)
      continue;

    cerr << "Got good stuff" << endl;

    RGBImage rgb_image;
    if (rgb_img)
      rgb_image =rgb_img->image();

    cerr << depth_img << " " << rgb_img  << endl;

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
    synchronizers[i].reset();
    float cpu = profiler->usageCounter()->totalCPUUsage();
    size_t mem = profiler->usageCounter()->totalMemory();
    if (vt) {
      printf("\r queue: %d, cpu: %f, mem: %ld, %s       ", (int) _messages->size(), cpu, mem, vt->lastMessage().c_str());
      fflush(stdout);
    }
  }
}

int main(int argc, char **argv) {
  std::string alignerType="projective";
  std::string config="Xtion320x240";
  std::string output_filename="";
  std::string odom_frame_id = "/odom";
  bool cam_only=false;
  float bad_points_ratio = 0.1;
  float damping = 100;
  float tbb = 3;
  float obb = 8;
  int shrink = 1;
  int skip = 0;
  std::string transforms_filename = "";
  std::string dump_filename = "";
  std::string base_link_frame_id = "";
  float max_distance = 3;
  float min_distance = 0;
  int c = 1;
  bool single = false;
  std::vector<std::string> depth_topics;
  std::vector<std::string> rgb_topics;
  
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
    else if (! strcmp(argv[c], "-base_link_frame_id")){
      c++;
      base_link_frame_id = argv[c];
    }
    else if (! strcmp(argv[c], "-shrink")){
      c++;
      shrink = atoi(argv[c]);
    } 
    else if (! strcmp(argv[c], "-t")){
      c++;
      depth_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-rgbt")){
      c++;
      rgb_topics.push_back(argv[c]);
    }
    else if (! strcmp(argv[c], "-bpr")){
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
    else if (! strcmp(argv[c], "-skip")){
      c++;
      skip = atoi(argv[c]);
    }
    else if (! strcmp(argv[c], "-odom_weight")){
      c++;
      odom_weight = atof(argv[c]);
      cerr << "odometry weight: " << odom_weight << std::endl;
    }

    else if (! strcmp(argv[c], "-o")){
      c++;
      output_filename = argv[c];
    }
    c++;
  }

  tf::TransformListener * listener = 0;
  std::vector<ImageMessageListener*> camera_listeners;
  
  ros::init(argc, argv, "fps_local_mapper_node");
  if (base_link_frame_id.length()>0){
    cerr << "making listener" << endl;
    listener = new tf::TransformListener(ros::Duration(60.0));
  }
  ros::NodeHandle nh;
  image_transport::ImageTransport itr(nh);

  synchronizers.resize(depth_topics.size());
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

  cerr << "resizing synchronizers" << endl;
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
  tracker->setFrameSkip(skip);  
  tracker->setImageShrink(shrink);
  ProjectiveAligner* aligner = dynamic_cast<ProjectiveAligner*>(&tracker->aligner());

  cerr << " Done" << endl;

  boss::Serializer* ser = 0;
  boss::IdContext * context = 0;
  if (output_filename != "") {
    ser  = new boss::Serializer();
    ser->setFilePath(output_filename);
    ser->setBinaryPath(output_filename + ".d/<classname>.<nameAttribute>.<id>.<ext>");
    context=ser;
  } else {
    context = new boss::IdContext;
  }

  if (dump_filename!= ""){
    writer=new MessageWriter;
    writer->open(dump_filename);
  }
  
  LocalMapperRos* local_map_maker = new LocalMapperRos(tracker, 1, ser, context);

  local_map_maker->setTrajectoryMaxTranslation(tbb);
  local_map_maker->setTrajectoryMaxOrientation(obb);

  vt = new VerboseTrigger(tracker, Tracker::PROCESSING_DONE, 0, 
		     "<seq> Time: <total_time>, FPS: <fps>, [cloud: <make_cloud_time>, alignment: <alignment_time>, validate: <validate_time>, merge: <merge_time>, tail: <tail_time>]");


  vt->setOutputStream(0);

  cerr << "ALL IN PLACE" << endl;
  
  sorter->setWriteBackEnabled(false);
  if (transforms_filename.length()){
    StaticTransformTree * transforms = 0;
    transforms = new StaticTransformTree;
    transforms->load(transforms_filename);
    TfOverriderTrigger* tf_overrider = new TfOverriderTrigger(sorter, 0, transforms);
  }
 
  MessageEnlisterTrigger* enlister = new MessageEnlisterTrigger(sorter, 10, _messages);
  
  SystemUsageCounter* usage_counter = new SystemUsageCounter;
  profiler = new ProfilerTrigger(tracker, Tracker::PROCESSING_DONE, 100, usage_counter);
  //CallTrackerTrigger* caller = new CallTrackerTrigger(&sorter, 1, tracker);
  for (std::vector<std::string>::iterator it = depth_topics.begin(); it!=depth_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->subscribe(topic);
    cerr << "subscribing to topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }
  
  for (std::vector<std::string>::iterator it = rgb_topics.begin(); it!=rgb_topics.end(); it++) {
    std::string topic = *it;
    ImageMessageListener* camera_listener = 
      new ImageMessageListener (&nh, &itr, sorter, listener, odom_frame_id, base_link_frame_id);
    camera_listener->subscribe(topic);
    cerr << "subscribing to topic: " << topic << endl;
    camera_listeners.push_back(camera_listener);
  }

  tf::TransformBroadcaster* broadcaster = new tf::TransformBroadcaster;

  fps_mapper::CloudPublisherTrigger* cloud_publisher = 
    new fps_mapper::CloudPublisherTrigger(tracker, 
  					  Tracker::PROCESSING_DONE,
  					  2, nh, broadcaster);
  local_map_maker->init(nh);
  pthread_t runner;
  thread_run  = true;
  pthread_create(&runner, 0, run_local_mapper, 0);
  ros::spin();
}


