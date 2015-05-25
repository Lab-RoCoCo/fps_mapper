#pragma once

#include "tracker.h"
#include "core/multi_projector.h"
#include "core/multi_camera_info.h"


namespace fps_mapper {

  class MultiTracker : public Tracker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class Trigger;

    // simple static factory, instantiates a tracker with a configuration
    static MultiTracker* makeTracker(const std::string& type, const std::string config);


    //! ctor if a = 0, it creates its own instance of Projective Aligner with a Pinhole projector;
    //! if you pass your aligner it will use that one
    //! this way you can construct a tracker based on different alignment policies
    MultiTracker(BaseAligner* a=0);

       
    //! processes a frame, froma raw depth image, updates the transform and adds the new colud to the local map
    //! @param depth : uint16_t depth image
    //! @param K: camera matrix
    //! @param depthSCale: the depth conversion to pass from pixel units to meters (for kinect 1e-3)
    //! @param seq: the seq field in the header of the ros Image message
    //! @param sensorOffset: the position of the camera on the base. 
    //! The pose offset is computed by the system in the reference frame of the base. This allows to handle multiple cams
    //! @param topic: the topic of the image
    virtual void processFrame(const RawDepthImage& depth,
			      const RGBImage& rgb,
			      const Eigen::Matrix3f& K, 
			      float depthScale, 
			      int seq,
			      double timestamp,
			      const std::string& topic = "/camera/depth/image_raw",
			      const std::string& frame_id = "/camera/depth/frame_id",
			      const Eigen::Isometry3f& sensor_offset=Eigen::Isometry3f::Identity(),
			      const Eigen::Isometry3f& odom_guess=Eigen::Isometry3f::Identity(),
			      const Matrix6f& odom_info=Matrix6f::Zero());

    void init(const std::vector<std::string>& topics);

    const std::vector<int>& lastSeqs() const {return _last_seqs;}
    std::vector<BaseCameraInfo*>& lastCameras() {return _last_cameras;}
  protected:
    int getIndex(const std::string& topic);
    void allocateImages(int rows, int cols);
    std::vector<int> _last_seqs;
    std::vector<BaseCameraInfo*> _last_cameras;
    bool projectorsReady();
    bool _projectors_ready;

    std::map<std::string, int> _topic2index;
    std::vector<RawDepthImage> _input_depths;
    std::vector<RGBImage> _input_rgbs;
    unsigned short* _depth_collage_buffer;
    cv::Vec3b* _rgb_collage_buffer;
    RawDepthImage _depth_collage;
    RGBImage _rgb_collage;

    MultiProjector* _multi_projector;

    bool _image_ready;
    int _image_stride;
    MultiCameraInfo _multi_camera;
    int _subimage_rows, _subimage_cols;
  };

}
