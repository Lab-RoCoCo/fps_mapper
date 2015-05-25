#include "calibration_measurement.h"

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;
  CalibrationMeasurement::CalibrationMeasurement(const Eigen::Isometry3f& base_motion,
			 const Eigen::Isometry3f& camera_motion,
			 Eigen::Isometry3f& camera_offset,
			 Eigen::Isometry3f& inverse_camera_offset):
    _base_motion(base_motion),
    _camera_motion(camera_motion), 
    _inverse_camera_motion(camera_motion.inverse()),
    _camera_offset(&camera_offset),
    _inverse_camera_offset(&inverse_camera_offset){
  }

}
