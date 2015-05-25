#include "extrinsic_calibrator_trigger.h"
#include "txt_io/static_transform_message.h"
#include "txt_io/message_reader.h"
#include "txt_io/message_writer.h"

#include "core/depth_utils.h"
#include "tracker/base_triggers.h"
#include "tracker_viewers/tracker_viewer.h"
#include "tracker/multi_tracker.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include "qglviewer.h"
#include "qapplication.h"

#include "calibration_measurement.h"
#include "calibration_prior.h"
#include "camera_calibration_solver.h"

namespace fps_mapper {

  using namespace std;
  using namespace txt_io;
  using namespace Eigen;



  ExtrinsicCalibratorTrigger::ExtrinsicCalibratorTrigger(Tracker* tracker,
							 int priorory): 
    Tracker::Trigger(tracker, Tracker::NEW_CAMERA_ADDED|Tracker::TRACKING_DONE,priorory){
    _os = &cout;
    _good_ratio = 0.2;
    _min_translation = 0.2;
    _min_rotation = 0.2;
    _min_measurements = 10;
    _iterations = 100;
    _tf_file_prefix = "";
    _num_calib_done = 0;
    _count = 0;
  }

  void ExtrinsicCalibratorTrigger::action(Tracker::TriggerEvent e) {
    if (e&Tracker::TRACKING_DONE){
      TransformPair current_transforms(_tracker->lastInitialGuess(), _tracker->globalT());

      CameraPoseMap::iterator it = _camera_poses.find(_tracker->lastCamera());
      if (it==_camera_poses.end()){
	_camera_poses.insert(make_pair(_tracker->lastCamera(), current_transforms));
	(*_os) << "#CAMERA " << _tracker->lastCamera() << " " << t2v(_tracker->lastCamera()->offset()).transpose() << endl;
	CameraCalibrationSolver* solver = new CameraCalibrationSolver(_tracker->lastCamera());
	_solvers.insert(make_pair(_tracker->lastCamera(), solver));


	float sensor_z = 0.5;
	Matrix6f info;
	info.setZero();
	info(2,2)=1e2;
	Eigen::Isometry3f mean;
	mean.setIdentity();
	mean.translation().z()=sensor_z;
	solver->addPrior(mean,info);

      } else {
	const Eigen::Isometry3f camera_offset = _tracker->lastCamera()->offset();
	TransformPair previous_transforms = it->second;
	Eigen::Isometry3f base_motion=previous_transforms.odometry.inverse()*current_transforms.odometry;
	Eigen::Isometry3f sensor_motion=
	  camera_offset.inverse()*previous_transforms.tracker_pose.inverse()*
	  current_transforms.tracker_pose*camera_offset;

	Eigen::Isometry3f odometry_sensor_motion=
	  camera_offset.inverse()*base_motion*camera_offset;
	
	if (_tracker->lastOutliersRatio()>_good_ratio) {
	  it->second = current_transforms;
	  cerr << "BAD_FRAME" << endl;
	  return;
	}

	float dl = base_motion.translation().norm();
	float dr = fabs(Eigen::AngleAxisf(base_motion.linear()).angle());
	if (dl>_min_translation || dr >_min_rotation) {
	  (*_os) << "CALIB_EDGE " 
		 << _tracker->lastCamera() << " "
		 << t2v(base_motion).transpose() << " "
		 << t2v(sensor_motion).transpose() << " " 
		 << _tracker->lastOutliersRatio() << endl;
	  CameraCalibrationSolver* solver = _solvers[_tracker->lastCamera()];
	  solver->addMeasurement(base_motion, sensor_motion);\
	  solver->_camera_offset = _tracker->lastCamera()->offset();
	  solver->_inverse_camera_offset = _tracker->lastCamera()->offset().inverse();
	  if (solver->_measurements.size()>_min_measurements) {
	    for (int i=0; i<100; i++) {
	      float cumulative_error = solver->oneRound();
	      if (i==0 || i==_iterations-1) {
		cerr << endl;
		cerr << "[ " << i << "] called solver, error: " << cumulative_error;
		cerr << " constraints: " << solver->_measurements.size();
		cerr << " error/constraint: " << cumulative_error/solver->_measurements.size();
		cerr << " T: " << t2v(solver->_camera_offset).transpose() << endl;
	      }	    
	    }
	    _tracker->lastCamera()->setOffset(solver->_camera_offset);
	    solver->_measurements.clear();
	    _num_calib_done ++;
	  } else {
	    cerr << endl << "We have still " << _min_measurements - solver->_measurements.size() << " to go" << endl;	      } 
	  it->second = current_transforms;
	}
	else {
	  cerr << "\rGOOD_FRAME dl: " << dl << " dr:" << dr << endl;
	}
      }
    }
    if (_num_calib_done==_solvers.size()){
      _num_calib_done=0;
      saveCurrentTransforms();
      _count++;
      _tracker->clearStatus();
    }
  }
  
  void ExtrinsicCalibratorTrigger::saveCurrentTransforms(){
    if (!_tf_file_prefix.length())
      return;
    MessageWriter writer;
    char filename [1024];
    sprintf(filename, "%s-%05d.txt", _tf_file_prefix.c_str(), _count);
    writer.open(filename);
    for (CameraPoseMap::iterator it = _camera_poses.begin(); it!=_camera_poses.end(); it++) {
      const BaseCameraInfo* cpm = it->first;
      StaticTransformMessage msg;
      msg.setTransform(cpm->offset());
      msg.setFromFrameId("/base_link");
      msg.setToFrameId(cpm->frameId());
      writer.writeMessage(msg);
    }
  }

}
