#pragma once
#include <string>
#include <map>
#include <tr1/memory>
#include "static_transform_message.h"
#include "base_sensor_message.h"
namespace txt_io{
  class StaticTransformTree {
  public:
    StaticTransformTree();
    inline const std::string rootFrameId() {return _root_frame_id; }
    void addMessage(StaticTransformMessage* msg);
    bool isWellFormed();
    bool getOffset(Eigen::Isometry3f &offset, const std::string& s);
    StaticTransformMessage* link(const std::string& s);
    typedef std::map<std::string, std::tr1::shared_ptr<StaticTransformMessage> > StringTransformMap;
    const StringTransformMap& tree() const {return _tree;}
    void applyTransform(BaseSensorMessage& msg);
    void load(const std::string& filename);
  protected:
    StringTransformMap _tree;
    std::string _root_frame_id;
  };
}
