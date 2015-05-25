#include "globals/system_utils.h"
#include "core/projective_aligner.h"
#include "core/pinhole_camera_info.h"
#include "core/multi_camera_info.h"
#include "multi_tracker.h"
#include "boss/serializer.h"
#include <tr1/memory>
#include <stdexcept>

#ifdef _FLANN_ENABLED_
#include "core/nn_aligner.h"
#endif

namespace fps_mapper {

  using namespace std;
  using namespace Eigen;
  using namespace system_utils;
  
  MultiTracker* MultiTracker::makeTracker(const std::string& type, const std::string config) {
    MultiTracker* dt=0;
    if (type == "projective") {
      ProjectiveAligner * al = new ProjectiveAligner(new MultiProjector);
      dt = new MultiTracker(al);
      al->setDefaultConfig(config);
      return dt;
    }
#ifdef _FLANN_ENABLED_
    if (type == "nn") {
      NNAligner * al = new NNAligner();
      dt = new MultiTracker(al);
      dt->setReferenceVoxelizeDensity(0.025);
      dt->setCurrentVoxelizeDensity(0.025);
      al->setIterations(10);
      return dt;
    }
#endif
    return 0;
  }


  MultiTracker::MultiTracker(BaseAligner* a): Tracker(a){
    if (!a) {
      _aligner=new ProjectiveAligner(new MultiProjector);
    } else {
      _aligner = a;
    }
    _projector = 0;
    _multi_projector = 0;
    _depth_collage_buffer = 0;
    _image_stride = 0;
    _topic2index.clear();
    _merging_distance = 0.1;
  }



  void MultiTracker::init(const std::vector<std::string>& topics) {
    if (_depth_collage_buffer)
      delete [] _depth_collage_buffer;

    for (std::vector<BaseCameraInfo*>::iterator it  = _multi_camera.cameraInfos().begin(); it!=_multi_camera.cameraInfos().end(); it++){
      delete *it;
    }
    _multi_camera.cameraInfos().clear();
    _topic2index.clear();


    for (size_t i = 0; i<topics.size(); i++){
      _topic2index.insert(make_pair(topics[i], i));
    }
    
    int num_images = _topic2index.size();
    _multi_camera.cameraInfos().resize(num_images);
    std::fill(_multi_camera.cameraInfos().begin(), _multi_camera.cameraInfos().end(), (BaseCameraInfo*)0);

    _multi_projector = new MultiProjector();
    _projector= _multi_projector;
    cerr << "done" << endl; 
    
    _image_stride = 0;
    _depth_collage_buffer = 0;
    _subimage_rows = 0;
    _subimage_cols = 0;
    _projectors_ready = false;

    _last_seqs.resize(_multi_camera.cameraInfos().size());
    _last_cameras.resize(_multi_camera.cameraInfos().size());
    
    cerr << "init done" << endl;
  }

  void MultiTracker::allocateImages(int rows, int cols) {
    if (_subimage_rows == rows && _subimage_cols == cols)
      return;

    if (_subimage_rows && (_subimage_rows!= rows || _subimage_cols!= cols))
      throw std::runtime_error ("the image sizes should be the same for all elements in collage");
    
    int num_images = _topic2index.size();
    _input_depths.resize(num_images);
    _input_rgbs.resize(num_images);
    _image_stride = rows*cols;

    _depth_collage_buffer = new unsigned short [_image_stride*num_images];
    _rgb_collage_buffer = new cv::Vec3b [_image_stride*num_images];

    for (size_t i=0; i<num_images; i++){
      _input_depths[i]=RawDepthImage(rows, cols, _depth_collage_buffer+(_image_stride*i));
      _input_rgbs[i]=RGBImage(rows, cols, _rgb_collage_buffer+(_image_stride*i));
    }
    _depth_collage=RawDepthImage(rows*num_images,cols, _depth_collage_buffer);
    _depth_collage=0;
    _rgb_collage=RGBImage(rows*num_images,cols, _rgb_collage_buffer);
    _rgb_collage=cv::Vec3b(0,0,0);

    _subimage_rows = rows;
    _subimage_cols = cols;
  }

  int MultiTracker::getIndex(const std::string& topic){
    std::map<std::string, int>::iterator it = _topic2index.find(topic);
    if(it==_topic2index.end())
      return -1;
    return it->second;
  }

  bool MultiTracker::projectorsReady(){
    if (_projectors_ready)
      return true;
    
    bool has_all_cams = true;
    for(size_t i = 0; i<_multi_camera.cameraInfos().size(); i++){
      has_all_cams = has_all_cams && _multi_camera.cameraInfos()[i];
    }
    if (! has_all_cams)
      return false;
    
    
    BaseCameraInfo* cam0 = _multi_camera.cameraInfos()[0];
    Eigen::Isometry3f cam0_offset = cam0->offset();
    Eigen::Isometry3f inv_cam0_offset = cam0_offset.inverse();

    for(size_t i = 0; i<_multi_camera.cameraInfos().size(); i++){
      BaseCameraInfo* cam=_multi_camera.cameraInfos()[i];
      cam->setOffset(inv_cam0_offset*cam->offset());
    }
    _multi_camera.setTopic("/multi_cam" + cam0->topic());
    _multi_camera.setFrameId(cam0->frameId());
    _multi_camera.setOffset(cam0_offset);
    _multi_projector->initFromCameraInfo(&_multi_camera);

    ProjectiveAligner* pal = dynamic_cast<ProjectiveAligner*>(_aligner);
    if (pal) {
      MultiProjector* mp = dynamic_cast<MultiProjector*>(&pal->projector());
      if (! mp)
	throw std::runtime_error("multipoint aligner in tracker does not have a multi projector");
      mp->initFromCameraInfo(&_multi_camera);
    }
    _projectors_ready=true;
    _cameras.addCamera(&_multi_camera);
    _last_camera = &_multi_camera;
    _last_camera_offset = _multi_camera.offset();
    callTriggers(NEW_CAMERA_ADDED);
    return true;
  }
    
  void MultiTracker::processFrame(const RawDepthImage& depth, 
				  const RGBImage& rgb,
				  const Eigen::Matrix3f& K, 
				  float depthScale, 
				  int seq, 
				  double timestamp,
				  const std::string& topic,
				  const std::string& frame_id,
				  const Eigen::Isometry3f& sensor_offset_, 
				  const Eigen::Isometry3f& odom_guess,
				  const Matrix6f& odom_info){

    // cerr << "A" << endl;
    int idx = getIndex(topic);
    if (idx<0)
      return;

    if (depth.cols == 0 && depth.rows == 0)
      return;


    // cerr << "B" << endl;
    int rows =depth.rows;
    int cols =depth.cols;
    if (_image_shrink>1) {
      rows/=_image_shrink;
      cols/=_image_shrink;
    } 
    allocateImages(rows,cols);

    // cerr << "C" << endl;
    Eigen::Matrix3f myK = K;
    _last_raw_depth=_input_depths[idx];
    _last_raw_rgb=_input_rgbs[idx];
    if (_image_shrink>1) {
      shrinkRawDepth(_last_raw_depth, depth, _image_shrink);
      //myK.block<2,3>(0,0)*= (1./_image_shrink);
      if (rgb.rows && rgb.cols)
	cv::resize(rgb, _last_raw_rgb,  cv::Size(0,0), 1./_image_shrink, 1./_image_shrink, cv::INTER_LINEAR);
    } else {
      memcpy(_last_raw_depth.data, depth.data, depth.rows*depth.cols*sizeof(unsigned short));
      if (rgb.rows && rgb.cols)
	memcpy(_last_raw_rgb.data, rgb.data, rgb.rows*rgb.cols*sizeof(unsigned short));
    }
    
    // cerr << "D" << endl;

    _last_seq = seq;
    _start_time  = getTime();
    _last_K = K;
    _last_depth_scale = depthScale;
    _last_topic = _multi_camera.topic();
    _last_timestamp = timestamp;

    // cerr << "E" << endl;

    BaseCameraInfo* current_subcamera = _cameras.getCamera(topic);
    bool new_camera = false;
    if (! current_subcamera) {
      current_subcamera = new PinholeCameraInfo(topic, frame_id, myK, sensor_offset_, depthScale);
      new_camera = true;
      _cameras.addCamera(current_subcamera);
      _multi_camera.cameraInfos()[idx]=current_subcamera;
      cerr << "adding camera " << idx << endl;
    }
    
    _last_raw_depth = _depth_collage;
    _last_seqs[idx] = seq;
    _last_cameras[idx] = current_subcamera;

    // cerr << "F" << endl;

    if (! projectorsReady()) 
      return;


    if (idx != 0)
      return;


    _frame_count ++;
    if (_frame_skip && (_frame_count%_frame_skip))
      return;

    _last_initial_guess = odom_guess;
    


    if (_current && _current!=_reference)
      delete _current;

    // cerr << "G" << endl;

   _make_cloud_time = getTime();
   _current = new  Cloud;
   _projector->setImageSize(_depth_collage.rows, _depth_collage.cols);
   std::tr1::shared_ptr<BaseCameraInfo> internal_multi_cam(_multi_camera.scale(1./_image_shrink));
   _projector->setCameraInfo(internal_multi_cam.get());
   _projector->unproject(*_current, _depth_collage, _rgb_collage);
   
   _current_time = getTime();

    _make_cloud_time = getTime() - _make_cloud_time;
    callTriggers(NEW_FRAME_CREATED);

    // cerr << "H" << endl;

   // make it sparse if necessary
    if (_current_voxelize_density>0){
      voxelize(*_current, _current_voxelize_density);
    }
    

    // cerr << "I" << endl;

    // if it is not the first cloud, do the work
    if (_reference ) {
      Eigen::Isometry3f odom_delta = _last_odom.inverse()*odom_guess;
      _alignment_time = getTime();
      _aligner->setReferenceModel(_reference);
      _aligner->setCurrentModel(_current);
      _aligner->align(odom_delta, odom_info);
	//cerr << "igi" << endl;
	//cerr << odom_info << endl;
      updateInformation();

      _alignment_time = getTime() - _alignment_time;

      _current_time = getTime();
      callTriggers(ALIGNMENT_DONE);

      _validate_time = getTime();
      // see if the track is broken by doing some reprojections anc omparing the images
      _current->transformInPlace(_aligner->T());
      reprojectBoth(_depth_collage.rows, _depth_collage.cols, _merging_gain);
      computeTrackBroken();
      /*
      cv::imshow("ref", _ref_buffer*0.1);
      cv::imshow("cur", _cur_buffer*0.1);
      cv::imshow("img", _depth_collage*10);
      cv::waitKey(0.1);
      */
      _validate_time = getTime() - _validate_time;
      
      _current_time = getTime();
      if (_track_broken) {
	callTriggers(TRACK_BROKEN);
	cerr << "track broken!" << endl;
      } else {
	callTriggers(TRACK_GOOD);
      }
    

      if (!_track_broken) {
	if (_merging_enabled) {
	  _merge_time = getTime();
	  merge(_ref_buffer, _ref_indices, *_reference, 
		_cur_buffer, _cur_indices, *_current, 
		_merging_distance);
	  _merge_time = getTime() - _merge_time;
	  
	}
      }
      _tail_time = getTime();
      _global_transform = _global_transform * _aligner->T();
      Eigen::Isometry3f iso=_aligner->T().inverse();
      _reference->transformInPlace(iso);
      _tail_time = getTime() - _tail_time;
      
    } 
    callTriggers(TRACKING_DONE);

    // cerr << "J" << endl;

    if (! _reference) {
      _reference = _current;
      _current_time = getTime();
      callTriggers(REFERENCE_FRAME_RESET);
    }
    _reference_offset = _multi_camera.offset();

    // cerr << "K" << endl;
  
    _current_time = getTime();
    callTriggers(PROCESSING_DONE);
    _last_odom = odom_guess;

    // cerr << "L" << endl;
  
  }


}
