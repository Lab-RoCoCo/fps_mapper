#pragma once
#include "ros_wrappers/ros_utils.h"
#include "fps_ros_msgs.h"

  
namespace fps_mapper {

  class LocalMapListener {
  public:
    LocalMapListener(boss::IdContext* context_=0);


    void init(ros::NodeHandle& n);
    virtual void onNewLocalMap(LocalMap* lmap);
    virtual void onNewNode(MapNode* node);
    virtual void onNewRelation(BinaryNodeRelation* rel);
    virtual void onNewCameraInfo(BaseCameraInfo* cam);

  protected:

    void pinholeCameraInfoCallback(const PinholeCameraInfoMsgConstPtr& msg);
    void multiCameraInfoCallback(const MultiCameraInfoMsgConstPtr& msg);
    void imageMapNodeCallback(const ImageMapNodeMsgConstPtr& msg);
    void multiImageMapNodeCallback(const MultiImageMapNodeMsgConstPtr& msg);
    void relationsCallback(const BinaryNodeRelationMsgConstPtr& msg);
    void localMapCallback(const LocalMapMsgConstPtr& msg);
    
    void processPendingCameraInfos();
    void processPendingImageNodes();
    void processPendingRelations();
    void processPendingLocalMaps();
    void processPendingMsgs();
    
    std::list<ImageMapNodeMsg> _pending_image_node_msgs;
    std::list<MultiImageMapNodeMsg> _pending_multi_image_node_msgs;
    std::list<PinholeCameraInfoMsg> _pending_camera_info_msgs;
    std::list<MultiCameraInfoMsg> _pending_multi_camera_info_msgs;
    std::list<LocalMapMsg> _pending_local_map_msgs;
    std::list<BinaryNodeRelationMsg> _pending_relations_msgs;

    ros::Subscriber _sub_camera_info, _sub_multi_camera_info;
    ros::Subscriber _sub_image_map_node, _sub_multi_image_map_node;
    ros::Subscriber _sub_local_map;
    ros::Subscriber _sub_relations;
    std::set<boss::Identifiable*> _objects;
    boss::IdContext* _context;
  };

}
