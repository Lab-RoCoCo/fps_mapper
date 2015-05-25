#pragma once
#include "boss/eigen_boss_plugin.h"
#include "core/multi_camera_info.h"
#include "map_node.h"

namespace fps_mapper {

  class MultiImageMapNode : public MapNode{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MultiImageMapNode(const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(), 
		 MultiCameraInfo* cam=0,
		 const std::string& topic = "",
		 int seq_=-1,
		 int id=-1,
		 boss::IdContext* context=0);

    inline const std::string& topic() const {return _topic; }
    inline void setTopic (const std::string topic_) {_topic = topic_;}

    inline int seq() const {return _seq;}
    inline void setSeq(int s)  { _seq = s; }

    inline MultiCameraInfo* cameraInfo() {return _camera_info;}
    inline void setCameraInfo(MultiCameraInfo* c)  { _camera_info = c;}
    virtual void draw(DrawAttributesType attributes = ATTRIBUTE_SHOW, int name = -1);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    std::vector<int>& subimageSeqs() { return _subimage_seqs; }
  protected:
    std::string _topic;
    MultiCameraInfo* _camera_info;
    int _seq;
    std::vector<int> _subimage_seqs;
  };

}
