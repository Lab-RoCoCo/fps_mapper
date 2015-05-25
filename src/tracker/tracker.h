#pragma once
#include "boss/serializer.h"
#include "core/projective_correspondence_finder.h"
#include "core/depth_utils.h"
#include "core/solver.h"
#include "core/base_aligner.h"
#include "core/base_camera_info.h"
#include "core/camera_info_manager.h"


#include <fstream>



namespace fps_mapper {

  class Tracker {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    friend class Trigger;

    // simple static factory, instantiates a tracker with a configuration
    static Tracker* makeTracker(const std::string& type, const std::string config);

    enum TriggerEvent  {
      NEW_FRAME_CREATED = 0x1,  // when making a new cloud of an image
      NEW_CAMERA_ADDED = 0x2,  // when making a new camera for the first time
      ALIGNMENT_DONE = 0x4,     // called after alignment
      TRACK_GOOD = 0x8,         // if the alignment succeeds
      TRACK_BROKEN = 0x10,       // if the alignment fails
      TRACKING_DONE = 0x20,      // after all has been done
      PROCESSING_DONE = 0x40,   // after processing, before the end of ptocessframe
      REFERENCE_FRAME_RESET = 0x80 // when the very first frame is created
    };

    //! class that defines a trigger, invoked when a specific event happens
    //! it registers himself on the tracker, responds to an eventl
    //! triggers for the same event are invoked according to their oriority (lower number, higher priority)
    class Trigger{
    public:
      Trigger(Tracker* tracker, int event, int priorory);
      virtual ~Trigger();
      virtual void action(TriggerEvent event) = 0;
      inline Tracker* tracker() { return _tracker;}
      inline const int event() const {return _event;}
      inline const int priority() const {return _priority;}
    protected:
      Tracker* _tracker;
      int _event;
      int _priority;
    };

    //! ctor if a = 0, it creates its own instance of Projective Aligner with a Pinhole projector;
    //! if you pass your aligner it will use that one
    //! this way you can construct a tracker based on different alignment policies
    Tracker(BaseAligner* a=0);
    
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

    //! access to the internal projector object
    BaseProjector& projector() { return *_projector;}

    //! access to the internal aligner object
    BaseAligner& aligner() { return *_aligner;}

    //! access to the internal CloudMaker object, used to make a cloud with normals
    //! from a depth image

    //! clears the internal state of the tracker
    //! starts a new local map using the current frame as initial frame
    //! keeps the old global transform unaltered
    void clearStatus();
    
    //! returns the current global transform estimate (depth-based odometry)
    inline const Eigen::Isometry3f globalT() const {return _global_transform;}

    //! returns the current array of camera infos
    //! a camera is created the first time an image message with an unseen topic is
    //! passed to processFrame()
    //! this object will contain a list of all cameras
    inline CameraInfoManager& cameras() { return _cameras; }


    //! sets/gets the parameters used to prune the density of the local map, after it has been
    //! augmented with the new cloud
    inline float referenceVoxelizeDensity() const {return _reference_voxelize_density;}  
    inline void setReferenceVoxelizeDensity(float vd)  {_reference_voxelize_density = vd;}

    //! sets/gets the parameters used to prune the density of a new cloud , after it has been
    //! created from the depth image
    inline float currentVoxelizeDensity() const {return _current_voxelize_density;}  
    inline void setCurrentVoxelizeDensity(float vd)  {_current_voxelize_density = vd;}

    //! access to the current cloud (the last one passed to the tracker)
    inline Cloud* currentModel() { return _current;}
    //! access to the reference cloud (the local map cloud)
    inline Cloud* referenceModel() { return _reference;}

    //! controls the integration of a new cloud
    //! if false, the robot pose is only tracked but not augmented
    inline void enableMerging (bool m) {_merging_enabled = m; }
    inline bool mergingEnabled () const { return _merging_enabled; }
  
    //! controls the fraction of outliers w.r.t he total points, after a track is claimed to be broken
    inline float badPointsRatio() const {return _bad_points_ratio;}
    inline void  setBadPointsRatio(float r) { _bad_points_ratio = r;}

    //! controls the maximum distance between a reprojected point and the map that makes it an inlier
    inline float inlierDistanceThreshold() const {return _inlier_distance_threshold;}
    inline void  setInlierDistanceThreshold(float t) { _inlier_distance_threshold = t;}

    //! points in a track that are closed than this threshold are claimed to be inliers
    inline float mergingDistance() const {return _merging_distance;}
    inline void  setMergingDistance(float t) { _merging_distance = t;}

    inline double startTime() const {return _start_time;}
    inline double currentTime() const {return _current_time;}
    inline double makeCloudTime() const {return _make_cloud_time;}
    inline double alignmentTime() const {return _alignment_time;}
    inline double validateTime() const {return _validate_time;}
    inline double mergeTime() const {return _merge_time;}
    inline double tailTime() const {return _tail_time;}

    inline int imageShrink() const {return _image_shrink;}
    inline void setImageShrink(int is)  { _image_shrink = is;}

    inline int lastSeq() const {return _last_seq;}
    inline RawDepthImage& lastRawDepth() {return _last_raw_depth;}
    inline const Eigen::Matrix3f& lastK() const {return _last_K; }
    inline float lastDepthScale() const {return _last_depth_scale;}
    inline bool isTrackBroken() const {return _track_broken;}
    inline BaseCameraInfo* lastCamera() {return _last_camera;}
    inline const std::string& lastTopic() const {return _last_topic;}
    inline const Eigen::Isometry3f& lastInitialGuess() const {return _last_initial_guess;}
    inline const Eigen::Isometry3f& lastCameraOffset() const {return _last_camera_offset;}
    inline float lastOutliersRatio() const {return _last_outliers_ratio;}
    inline double lastTimestamp() const { return _last_timestamp;}
    void setMaxDistance(float max_distance);
    void setMinDistance(float min_distance);

    virtual void setFrameSkip(int skip);
    inline int frameSkip() const {return _frame_skip;}
    inline int frameCount() const {return _frame_count;}
    void resetInformationMatrix();
    inline const Matrix6f& informationMatrix() const { return _information_matrix; }
  protected:
    typedef std::map<int, Trigger*> PriorityTriggerMap;
    typedef std::map<TriggerEvent, PriorityTriggerMap> EventTriggeMap;

    int _image_shrink;
    int _frame_skip;
    int _frame_count;
    CameraInfoManager _cameras;
    BaseProjector* _projector;
    BaseAligner* _aligner;
  
    Cloud* _reference;
    Cloud* _current;
    Eigen::Isometry3f _reference_offset;
    float _merging_gain;
    FloatImage _zbuffer;
    IndexImage _indices;
    Eigen::Isometry3f _global_transform;
    int _total_points;
    float _reference_voxelize_density;
    float _current_voxelize_density;
    bool _merging_enabled;
    float _bad_points_ratio;
    float _inlier_distance_threshold;
    float _merging_distance;

    void updateInformation();

    void callTriggers(TriggerEvent event);
    EventTriggeMap _triggers;

    double _start_time;
    double _current_time;
    double _make_cloud_time;
    double _alignment_time;
    double _validate_time;
    double _merge_time;
    double _tail_time;
    bool _track_broken;
    float _last_outliers_ratio;
    RawDepthImage _last_raw_depth;
    RGBImage _last_raw_rgb;

    Eigen::Matrix3f _last_K;
    int _last_seq;
    float _last_depth_scale;
    BaseCameraInfo* _last_camera;
    Eigen::Isometry3f _last_odom;
    double _last_timestamp;
  
    IndexImage _cur_indices, _ref_indices;
    FloatImage _cur_buffer, _ref_buffer;
    void reprojectBoth(int rows, int cols, float scale);
    void computeTrackBroken();
    std::string _last_topic;
    Eigen::Isometry3f _last_initial_guess;
    Eigen::Isometry3f _last_camera_offset;
    bool _information_matrix_reset;
    Matrix6f _information_matrix;
    Eigen::Isometry3f _information_matrix_origin;
 };

}
