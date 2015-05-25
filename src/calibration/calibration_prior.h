#pragma once
#include "calibration_measurement.h"

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;

  struct CalibrationPrior{
    CalibrationPrior(const Eigen::Isometry3f& camera_offset_mean,
		     const Matrix6f& camera_offset_info,
		     Eigen::Isometry3f& camera_offset,
		     Eigen::Isometry3f& inverse_camera_offset);
  
    inline Matrix6f omega() const {
      Eigen::Matrix3f R = _inverse_camera_offset_mean.linear();
      R.transposeInPlace();
      Matrix6f info;
      info.setZero();
      info.block<3,3>(0,0)=R*_camera_offset_info.block<3,3>(0,0)*R.transpose();
      info.block<3,3>(3,3)=R*_camera_offset_info.block<3,3>(3,3)*R.transpose();
      return info;
    }

    Eigen::Isometry3f* _camera_offset;
    Eigen::Isometry3f* _inverse_camera_offset;

    Eigen::Isometry3f _camera_offset_mean;
    Eigen::Isometry3f _inverse_camera_offset_mean;

    Matrix6f _camera_offset_info;

    inline Vector6f error(const Vector6f& delta_x=Vector6f::Zero()) const {
      Eigen::Isometry3f delta_X=v2t(delta_x);
      return t2v(_inverse_camera_offset_mean * (*_camera_offset) * delta_X);
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

  typedef std::vector<CalibrationPrior, Eigen::aligned_allocator<CalibrationPrior> > CalibrationPriorVector;

}
