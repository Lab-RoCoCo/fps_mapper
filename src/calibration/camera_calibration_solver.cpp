#include "camera_calibration_solver.h"

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;

  CameraCalibrationSolver::CameraCalibrationSolver(BaseCameraInfo *cam) {
    _cam = cam;
    _camera_offset=cam->offset();
    _inverse_camera_offset = _camera_offset.inverse();
    _damping = 1;
  }

  void CameraCalibrationSolver::addMeasurement(const Eigen::Isometry3f& base_motion,
					       const Eigen::Isometry3f& camera_motion){
    _measurements.push_back(CalibrationMeasurement(base_motion, camera_motion, _camera_offset, _inverse_camera_offset) );
  }

  void CameraCalibrationSolver::addPrior(const Eigen::Isometry3f& prior_mean,
					 const Matrix6f& prior_info){
    _priors.push_back(CalibrationPrior(prior_mean, 
				       prior_info, 
				       _camera_offset, 
				       _inverse_camera_offset) );
  }

  float CameraCalibrationSolver::oneRound(){
    float current_damping = _damping * _measurements.size();
    float cumulative_error = 0;
    Vector6f b = Vector6f::Zero();
    Matrix6f H = Matrix6f::Zero();
    for(size_t i=0;i<_measurements.size(); i++){	
      Vector6f e = _measurements[i].error();
      Matrix6f J = _measurements[i].jacobian();
      cumulative_error+=e.squaredNorm();
      H+=J.transpose()*J;
      b+=J.transpose()*e;
    }

    for(size_t i=0;i<_priors.size(); i++){	
      Vector6f e = _priors[i].error();
      Matrix6f J = _priors[i].jacobian();
      const Matrix6f& omega  = _priors[i].omega();
      float prior_error = e.transpose()* omega *e;
      //cerr << "prior_error: " << _priors[i]._camera_offset_mean.translation().z() -  _camera_offset.translation().z() <<  " " << e(2) <<  " " << _priors[i]._camera_offset_mean.translation().z() << endl;
      
      cumulative_error+=prior_error;
      H+=J.transpose()*omega*J;
      b+=J.transpose()*omega*e;
    }

    H += current_damping*Matrix6f::Identity();
    Vector6f dt = H.ldlt().solve(-b);
    _camera_offset = _camera_offset*v2t(dt);
    Eigen::Matrix3f R = _camera_offset.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _camera_offset.linear() -= 0.5 * R * E;
    
    _inverse_camera_offset = _camera_offset.inverse();
    return cumulative_error;
  }

}
