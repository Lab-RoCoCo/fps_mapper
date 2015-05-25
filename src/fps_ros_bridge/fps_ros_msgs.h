#include "ros_wrappers/ros_utils.h"
#include "boss/id_context.h"

#include "fps_map/local_map.h"
#include "fps_map/map_node.h"
#include "fps_map/image_map_node.h"
#include "fps_map/multi_image_map_node.h"
#include "fps_map/binary_node_relation.h"
#include "core/base_camera_info.h"
#include "core/pinhole_camera_info.h"
#include "core/multi_camera_info.h"

#include "fps_mapper/RichPointMsg.h"
#include "fps_mapper/CloudMsg.h"
#include "fps_mapper/PinholeCameraInfoMsg.h"
#include "fps_mapper/MultiCameraInfoMsg.h"
#include "fps_mapper/MapNodeMsg.h"
#include "fps_mapper/ImageMapNodeMsg.h"
#include "fps_mapper/MultiImageMapNodeMsg.h"
#include "fps_mapper/LocalMapMsg.h"
#include "fps_mapper/BinaryNodeRelationMsg.h"


namespace fps_mapper{

  void msg2cloud(fps_mapper::Cloud& dest,    const fps_mapper::CloudMsg& src);  
  void cloud2msg(fps_mapper::CloudMsg& dest, const fps_mapper::Cloud& src);

  PinholeCameraInfo* msg2pinholeCameraInfo(const PinholeCameraInfoMsg& msg, boss::IdContext* context);
  PinholeCameraInfoMsg pinholeCameraInfo2msg(PinholeCameraInfo* src, boss::IdContext*  context);

  
  MultiCameraInfo* msg2multiCameraInfo(const MultiCameraInfoMsg& msg, boss::IdContext* context);
  MultiCameraInfoMsg multiCameraInfo2msg(MultiCameraInfo* src, boss::IdContext*  context);

  MapNode* msg2MapNode(const MapNodeMsg& msg, boss::IdContext* context);
  MapNodeMsg mapNode2msg(MapNode* src, boss::IdContext* context);

  ImageMapNode* msg2imageMapNode(const ImageMapNodeMsg& msg, boss::IdContext* context);
  ImageMapNodeMsg imageMapNode2msg(ImageMapNode* src, boss::IdContext* context);

  MultiImageMapNode* msg2multiImageMapNode(const MultiImageMapNodeMsg& msg, boss::IdContext* context);
  MultiImageMapNodeMsg multiImageMapNode2msg(MultiImageMapNode* src, boss::IdContext* context);

  LocalMap* msg2localMap(const LocalMapMsg& msg, boss::IdContext* context);
  LocalMapMsg localMap2msg(LocalMap* src, boss::IdContext* context);
  
  BinaryNodeRelation* msg2binaryNodeRelation(const BinaryNodeRelationMsg& msg, boss::IdContext* context);
  BinaryNodeRelationMsg binaryNodeRelation2msg(BinaryNodeRelation* src, boss::IdContext* context);
  
}
