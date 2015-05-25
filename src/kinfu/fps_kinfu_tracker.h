#pragma once

#include <pcl/gpu/kinfu_large_scale/kinfu.h>

#include "globals/defs.h"

#include "core/cloud.h"
#include "core/base_projector.h"

namespace fps_mapper {

  class FPSKinfuTracker: public pcl::gpu::kinfuLS::KinfuTracker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class Trigger;

    FPSKinfuTracker(const Eigen::Vector3f &volume_size, const float shifting_distance,
		    int rows = 240, int cols = 320);
    
    virtual void processFrame(const RawDepthImage& depth, const Eigen::Matrix3f& K, 
			      float depth_scale, int image_shrink = 1,
			      const std::string& topic = "/camera/depth/image_raw",
			      const std::string& frame_id = "/camera/depth/frame_id",
			      const Eigen::Isometry3f& sensor_offset = Eigen::Isometry3f::Identity(), 
			      const Eigen::Isometry3f& odom_guess = Eigen::Isometry3f::Identity()); 

    void shrinkAndConvertRawDepth(pcl::gpu::PtrStepSz<const unsigned short>& dest_buffer, 
				  RawDepthImage& dest_raw_buffer,
				  const RawDepthImage& src_buffer, 
				  int image_shrink);
    void reprojectBoth(int rows, int cols, float scale);
    
    void setMaxDistance(float max_distance) { if(_projector) { _projector->setMaxDistance(max_distance); } }
    void setMinDistance(float min_distance) { if(_projector) { _projector->setMinDistance(min_distance); } }

    inline const Eigen::Isometry3f& globalTransform() const { return _last_global_transform; };
    inline const Eigen::Isometry3f& deltaTransform() const { return _last_delta_transform; };
    inline float mergingDistance() const {return _merging_distance;}
    inline Cloud* referenceModel() { return _reference_model; }
    inline Cloud* currentModel() { return _current_model; }
    inline BaseProjector& projector() { return *_projector; }

    inline void setMergingDistance(float merging_distance) { _merging_distance = merging_distance; }
    
  protected:			      
    Eigen::Isometry3f _last_odom;
    Eigen::Isometry3f _last_delta_transform;
    Eigen::Isometry3f _last_global_transform;
    Eigen::Isometry3f _new_global_transform;
    std::vector<unsigned short> _source_depth_data;
    pcl::gpu::PtrStepSz<const unsigned short> _last_depth;
    KinfuTracker::DepthMap _depth_device;

    float _merging_distance;
    RawDepthImage _last_raw_depth;
    IndexImage _cur_indices, _ref_indices;
    FloatImage _cur_buffer, _ref_buffer;
    Cloud* _reference_model;
    Cloud* _current_model;
    BaseProjector* _projector;
  };

}
