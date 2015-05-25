#pragma once
#include "calibration_measurement.h"
#include "calibration_prior.h"
#include "camera_calibration_solver.h"
#include "tracker/base_triggers.h"
#include <Eigen/Core>
#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>



namespace fps_mapper {

  using namespace std;
  using namespace Eigen;



  class ExtrinsicCalibratorTrigger: public Tracker::Trigger{

    struct TransformPair{
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      TransformPair(){};
      TransformPair(const Eigen::Isometry3f& odometry, 
		    const Eigen::Isometry3f& tracker_pose){
	this->odometry= odometry;
	this->tracker_pose = tracker_pose;
      }
      Eigen::Isometry3f odometry;
      Eigen::Isometry3f tracker_pose;
    };
    typedef std::map<BaseCameraInfo*, TransformPair, std::less <BaseCameraInfo*>, Eigen::aligned_allocator< std::pair<BaseCameraInfo*, TransformPair> > > CameraPoseMap;


  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ExtrinsicCalibratorTrigger(Tracker* tracker,
			       int priorory);

    inline void setOutputStream(ostream& os) {_os = &os;}

    inline void setTfFilePrefix(const std::string& s) { _tf_file_prefix = s;}

    virtual void action(Tracker::TriggerEvent e);  

    void saveCurrentTransforms();
		      
  protected:
    std::string _tf_file_prefix;
    CameraPoseMap _camera_poses;
    ostream* _os;
    float _good_ratio;
    float _min_translation;
    float _min_rotation;
    int _min_measurements;
    int _iterations;
    int _num_calib_done;
    int _count;
    std::map<BaseCameraInfo*, CameraCalibrationSolver*> _solvers;
  };

}
