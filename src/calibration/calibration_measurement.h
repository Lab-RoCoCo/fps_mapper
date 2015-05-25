#pragma once

#include "globals/defs.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>
#include <deque>
#include <queue>
#include <vector>


namespace fps_mapper {

using namespace std;
using namespace Eigen;

struct CalibrationMeasurement{
  CalibrationMeasurement(const Eigen::Isometry3f& base_motion,
			 const Eigen::Isometry3f& camera_motion,
			 Eigen::Isometry3f& camera_offset,
			 Eigen::Isometry3f& inverse_camera_offset);

  Eigen::Isometry3f _base_motion;
  Eigen::Isometry3f _camera_motion;
  Eigen::Isometry3f _inverse_camera_motion;    
  Eigen::Isometry3f* _camera_offset;
  Eigen::Isometry3f* _inverse_camera_offset;

  inline Vector6f error(const Vector6f& delta_x=Vector6f::Zero()) const {
    Eigen::Isometry3f delta_X=v2t(delta_x);
    Eigen::Isometry3f inverse_delta_X=v2t(delta_x).inverse();
    return t2v(_inverse_camera_motion * inverse_delta_X * (*_inverse_camera_offset) * _base_motion * (*_camera_offset) * delta_X);
  }

  inline Matrix6f jacobian()  const {
    float epsilon = 1e-2;
    Matrix6f J;
    J.setZero();
    float two_inverse_epsilon = .5/epsilon;
    for (int i = 0; i<6; i++){
      Vector6f increment=Vector6f::Zero();
      increment(i) = epsilon;
      J.col(i)=two_inverse_epsilon * (error(increment) - error(-increment));
    }
    return J;
  }

};

typedef std::vector<CalibrationMeasurement, Eigen::aligned_allocator<CalibrationMeasurement> > CalibrationMeasurementVector;

}
