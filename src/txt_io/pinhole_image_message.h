#pragma once
#include <string>
#include <iostream>
#include "base_image_message.h"
#include <Eigen/Core>

namespace txt_io {

  class PinholeImageMessage: public BaseImageMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PinholeImageMessage(const std::string& topic="", const std::string& frame_id="", int seq=-1, double timestamp=-1);
    virtual const std::string& tag() const;
    virtual void fromStream(std::istream& is);
    virtual void toStream(std::ostream& os) const;
    inline const Eigen::Matrix3f& cameraMatrix() const {return _camera_matrix; }
    inline void setCameraMatrix(const Eigen::Matrix3f& camera_matrix) {_camera_matrix = camera_matrix;}
  protected:
    static const std::string _tag;
    Eigen::Matrix3f _camera_matrix;
  };

}
