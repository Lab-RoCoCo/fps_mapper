#pragma once

#include "core/depth_utils.h"
#include "core/nn_aligner.h"
#include "core/projective_aligner.h"
#include "tracker/tracker.h"
#include "tracker/base_triggers.h"
#include "local_mapper/local_map_triggers.h"
#include "tracker/multi_tracker.h"
#include "txt_io/tf_overrider_trigger.h"
#include "ros_wrappers/image_message_listener.h"
#include "txt_io/message_enlister_trigger.h"
#include "txt_io/message_dumper_trigger.h"
#include "txt_io/pinhole_image_message.h"
#include "fps_ros_msgs.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf/transform_listener.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <vector>
#include <fstream>
#include <iostream>

namespace fps_mapper {

  class LocalMapperRos : public LocalMapTrigger{
  public:
    LocalMapperRos(Tracker* tracker,
		   int priorory = 10,
		   boss::Serializer* ser=0,
		   boss::IdContext * context=0):
  
      LocalMapTrigger(tracker,
		      Tracker::TRACK_GOOD|
		      Tracker::TRACK_BROKEN|
		      Tracker::REFERENCE_FRAME_RESET|
		      Tracker::TRACKING_DONE|
		      Tracker::NEW_CAMERA_ADDED,
		      priorory,
		      ser){
      _context = context;
      cerr << "LOCAL MAPPER ROS" << endl;
    }
  

    void init(ros::NodeHandle& nh);
    virtual void onCameraInfoCreated(BaseCameraInfo* camera_info);
    virtual void onLocalMapCreated(LocalMap* lmap);
    virtual void onNewNodeCreated(MapNode* node, BinaryNodeRelation* rel);
    virtual void onRelationCreated(BinaryNodeRelation* rel);



  protected:
    void serializeCameras();
    void serializeCamera(BaseCameraInfo* cam);
    void serializeTrajectory(MapNodeList& nodes);
    void serializeNode(MapNode* n);
    void serializeRelation(BinaryNodeRelation* rel);
    std::vector<BaseCameraInfo*> _camera_infos;
    ros::Publisher _image_node_pub;
    ros::Publisher _multi_image_node_pub;
    ros::Publisher _camera_info_pub;
    ros::Publisher _multi_camera_info_pub;
    ros::Publisher _local_map_pub;
    ros::Publisher _relations_pub;
    boss::IdContext* _context;
  };
}
