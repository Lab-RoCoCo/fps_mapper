#pragma once
#include "base_message.h"
#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace txt_io {

  class BaseSensorMessage: public BaseMessage {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BaseSensorMessage(const std::string& topic="", const std::string& frame_id="",int seq=-1, double timestamp=-1);
    virtual void fromStream(std::istream& is);
    virtual void  toStream(std::ostream& os) const;
    inline bool hasOdom() const  {return _has_odom;}
    inline bool hasImu() const  {return _has_imu;}
    inline const Eigen::Isometry3f& offset() const {return _offset;}
    inline void setOffset(const Eigen::Isometry3f& offset_) { _offset = offset_;}
    inline const Eigen::Isometry3f& odometry() const {return _odometry; }
    inline void setOdometry(const Eigen::Isometry3f& odometry_) { _odometry = odometry_; _has_odom = true;}
    inline const Eigen::Isometry3f& imu() const {return _imu; }
    inline void setImu(const Eigen::Isometry3f& imu_) { _imu = imu_; _has_imu = true;}
    inline const std::string& topic() const {return _topic;}
    inline int seq() const {return _seq;}
    inline double timestamp() const {return _timestamp;}
    inline void  setTopic(std::string& t)  {_topic = t;}
    inline const std::string& frameId() const {return _frame_id;}
    inline void setFrameId(const std::string& fid) {_frame_id = fid;}
    inline void setSeq(int s)  {_seq = s;}
    inline void setTimestamp(double t)  {_timestamp=t;}
  protected:
    std::string _topic;
    std::string _frame_id;
    int _seq;
    double _timestamp;
    bool _has_imu;
    bool _has_odom;
    Eigen::Isometry3f _offset;
    Eigen::Isometry3f _odometry;
    Eigen::Isometry3f _imu;
  };

}






