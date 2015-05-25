#pragma once
#include "base_message.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace txt_io {

  class StaticTransformMessage: public BaseMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class StaticTransformTree;
    StaticTransformMessage(const std::string& from_frame_id = "", 
		     const std::string& to_frame_id="", 
		     const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity());
    virtual const std::string& tag() const;
    virtual void fromStream(std::istream& is);
    virtual void  toStream(std::ostream& os) const;
    inline const std::string& fromFrameId() const {return _from_frame_id;}
    inline void setFromFrameId(const std::string& id)  {_from_frame_id = id;}
    inline const std::string& toFrameId() const {return _to_frame_id;}
    inline void setToFrameId(const std::string& id)  {_to_frame_id = id;}
    inline const Eigen::Isometry3f& transform() const {return _transform;}
    inline void setTransform(const Eigen::Isometry3f& transform_) { _transform = transform_;}    
  protected:
    inline StaticTransformMessage *parent() {return _parent;}
    inline void setParent(StaticTransformMessage * parent_) {_parent = parent_;}
 
    static const std::string _tag;
    std::string _from_frame_id, _to_frame_id;
    Eigen::Isometry3f _transform;
    StaticTransformMessage* _parent;
  };

}






