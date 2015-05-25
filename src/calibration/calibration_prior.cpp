#include "calibration_prior.h"

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;
  
  CalibrationPrior::CalibrationPrior(const Eigen::Isometry3f& camera_offset_mean,
		     const Matrix6f& camera_offset_info,
		     Eigen::Isometry3f& camera_offset,
		     Eigen::Isometry3f& inverse_camera_offset):
      _camera_offset(&camera_offset),
      _inverse_camera_offset(&inverse_camera_offset),
      _camera_offset_mean(camera_offset_mean),
      _camera_offset_info(camera_offset_info)
    {
      _inverse_camera_offset_mean = _camera_offset_mean.inverse();
    }
  
}
