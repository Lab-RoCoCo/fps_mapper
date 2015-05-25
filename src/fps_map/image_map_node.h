#pragma once
#include "boss/eigen_boss_plugin.h"
#include "core/base_camera_info.h"
#include "map_node.h"

namespace fps_mapper {

  class ImageMapNode : public MapNode{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImageMapNode(const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(), 
		 BaseCameraInfo* cam=0,
		 const std::string& topic = "",
		 int seq_=-1,
		 int id=-1,
		 boss::IdContext* context=0);

    inline const std::string& topic() const {return _topic; }
    inline void setTopic (const std::string topic_) {_topic = topic_;}

    inline int seq() const {return _seq; }
    inline void setSeq(int s)  { _seq = s; }

    inline BaseCameraInfo* cameraInfo() {return _camera_info;}
    inline void setCameraInfo(BaseCameraInfo* c)  { _camera_info = c;}
    virtual void draw(DrawAttributesType attributes = ATTRIBUTE_SHOW, int name = -1);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

  protected:
    std::string _topic;
    BaseCameraInfo* _camera_info;
    int _seq;
  };

}
