#include "fps_kinfu_tracker.h"

#include "core/pinhole_projector.h"
#include "core/pinhole_camera_info.h"

namespace fps_mapper {

  FPSKinfuTracker::FPSKinfuTracker(const Eigen::Vector3f &volume_size, const float shifting_distance,
				   int rows, int cols): 
    pcl::gpu::kinfuLS::KinfuTracker(volume_size, shifting_distance, rows, cols) {
    _last_odom.setIdentity();
    _last_global_transform.setIdentity();
    _last_global_transform.translation() = getLastEstimatedPose().translation();    
    _last_global_transform.linear() = getLastEstimatedPose().rotation();
    _new_global_transform.setIdentity();
    _last_delta_transform.setIdentity();
    _merging_distance = 0.1f;
    _reference_model = 0;
    _current_model = 0;
    _projector = new PinholeProjector();
  }
 
  void FPSKinfuTracker::reprojectBoth(int rows, int cols, float scale) {
    _projector->pushState();
    _projector->setImageSize(scale*rows, scale*cols);
    _projector->scaleCamera(scale);
    _projector->project(_cur_buffer, _cur_indices, Eigen::Isometry3f::Identity(), *_current_model);
    _projector->project(_ref_buffer, _ref_indices, Eigen::Isometry3f::Identity(), *_reference_model);
    _projector->popState();
  }

  void FPSKinfuTracker::processFrame(const RawDepthImage& depth, const Eigen::Matrix3f& K, 
				     float depth_scale, int image_shrink, 
				     const std::string& topic, const std::string& frame_id,
				     const Eigen::Isometry3f& sensor_offset, 
				     const Eigen::Isometry3f& odom_guess) {    
    // If input is not valid return
    if(depth.cols == 0 || depth.rows == 0) { return; }

    // Shrink and convert input depth image
    shrinkAndConvertRawDepth(_last_depth, _last_raw_depth, depth, image_shrink);
    Eigen::Matrix3f shrinked_K = K;
    shrinked_K.block<2, 3>(0, 0) *= (1.0f / (float)image_shrink);


    // Create models
    BaseCameraInfo* camera_info;
    camera_info = new PinholeCameraInfo(topic, frame_id, shrinked_K, sensor_offset, depth_scale);    
    if(_current_model && _current_model != _reference_model) { delete _current_model; }
    _current_model = new Cloud;
    _projector->setImageSize(_last_raw_depth.rows, _last_raw_depth.cols);
    _projector->setCameraInfo(camera_info);
    _projector->unproject(*_current_model, _last_raw_depth);
    delete camera_info;

    // Align
    if(_reference_model) {
      reprojectBoth(_last_raw_depth.rows, _last_raw_depth.cols, 0.5);
      Eigen::Isometry3f odom_delta = _last_odom.inverse() * odom_guess;
      _depth_device.upload(_last_depth.data, _last_depth.step, _last_depth.rows, _last_depth.cols);
      bool has_image = (*this)(_depth_device);
      _new_global_transform = _last_global_transform;
      _last_global_transform.translation() = getLastEstimatedPose().translation();
      _last_global_transform.linear() = getLastEstimatedPose().rotation();
      _last_delta_transform = _new_global_transform.inverse() * _last_global_transform; 

      if(icpIsLost()) { std::cout << "[INFO]: kinfu lost" << std::endl; }
      
      _current_model->transformInPlace(_last_delta_transform);
      
      merge(_ref_buffer, _ref_indices, *_reference_model, 
      	    _cur_buffer, _cur_indices, *_current_model, 
      	    _merging_distance);
      std::cout << "[DEBUG]: reference model size " << _reference_model->size() << std::endl;

      _reference_model->transformInPlace(_last_delta_transform.inverse());      

    }
    else { _reference_model = _current_model; }
    
    std::cout << "[INFO] delta  transform " << t2v(_last_delta_transform).transpose() << std::endl;    
    std::cout << "[INFO] global transform " << t2v(_last_global_transform).transpose() << std::endl;    
    
    _last_odom = odom_guess;
  }						
 
  void FPSKinfuTracker::shrinkAndConvertRawDepth(pcl::gpu::PtrStepSz<const unsigned short>& dest_buffer, 
						 RawDepthImage& dest_raw_buffer,
						 const RawDepthImage& src_buffer, 
						 int image_shrink) {
    int rows = src_buffer.rows;
    int cols = src_buffer.cols;   
    if(rows % image_shrink) { throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows"); }
    if(cols % image_shrink) { throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols"); }

    int drows = rows / image_shrink;
    int dcols = cols / image_shrink;

    dest_buffer.cols = dcols;
    dest_buffer.rows = drows;
    dest_buffer.step = dest_buffer.cols * dest_buffer.elemSize();
    _source_depth_data.resize(dest_buffer.cols * dest_buffer.rows);   
    std::fill(_source_depth_data.begin(), _source_depth_data.end(), 0);
    dest_raw_buffer.create(drows, dcols);
    dest_raw_buffer = 0;

    // avoid divisions and use a lookup table
    int lv = rows > cols ? rows:cols;
    int ttable[lv];
    for(int i = 0; i < lv; i++) { ttable[i] = i / image_shrink; }

    for(int r = 0; r < rows; r++) {
      const unsigned short* src_z_ptr = src_buffer.ptr<unsigned short>(r);
      int dr = ttable[r];
      unsigned short* dest_z_ptr = (unsigned short*) &_source_depth_data[r * cols];  
      unsigned short* dest_raw_z_ptr = dest_raw_buffer.ptr<unsigned short>(dr);
      int cc = 0;
      for(int c = 0; c < cols; c++) {
	unsigned short src_z = *src_z_ptr;
	src_z_ptr++;
	if(src_z == 0) { continue; }
	unsigned short& dest_z = *(dest_z_ptr + ttable[c]);
        unsigned short& dest_raw_z = *(dest_raw_z_ptr + ttable[c]);
	if(!dest_z || dest_z < src_z) { 
          dest_z = src_z; 
          dest_raw_z = src_z; 
        }
      }
    }
    dest_buffer.data = &_source_depth_data[0];      
  }

}
