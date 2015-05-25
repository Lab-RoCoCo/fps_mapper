#pragma once
#include "base_message.h"
#include "../playground/bbox.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>



namespace txt_io {
  
  
  class BaseBBoxMessage : public BaseMessage {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      BaseBBoxMessage( const fps_mapper::BoundingBox &box ) : _bbox(box) {
      };
      virtual void fromStream(std::istream& is);
      virtual void  toStream(std::ostream& os) const;
      
    protected:
      fps_mapper::BoundingBox _bbox;
      /*
      /// size of the bounding box (x, y, z)
      Eigen::Vector3f _size;
      /// transformation matrix that moves the bounding box into place
      Eigen::Isometry3f _iso;
      */
  }
  
  
} // end of namespace txt_io

