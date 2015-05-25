#pragma once
#include "calibration_measurement.h"
#include "calibration_prior.h"
#include "core/base_camera_info.h"

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;


  struct CameraCalibrationSolver {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    CameraCalibrationSolver(BaseCameraInfo *cam) ;
    void addMeasurement(const Eigen::Isometry3f& base_motion,
			const Eigen::Isometry3f& camera_motion);
    void addPrior(const Eigen::Isometry3f& prior_mean,
		  const Matrix6f& prior_info);
    float oneRound();

    BaseCameraInfo* _cam;
    CalibrationMeasurementVector _measurements;
    CalibrationPriorVector _priors;
    Eigen::Isometry3f _camera_offset;
    Eigen::Isometry3f _inverse_camera_offset;
    float _damping;

  };

}
