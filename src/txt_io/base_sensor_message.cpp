#include "base_sensor_message.h"
#include "globals/defs.h"
#include <cstdio>
#include <cstring>
#include "opencv2/opencv.hpp"


namespace txt_io {
  using namespace std;
 
  BaseSensorMessage::BaseSensorMessage(const std::string& topic_, const std::string& frame_id, int seq_, double timestamp_):
    _topic (topic_), _frame_id(frame_id), _seq(seq_), _timestamp(timestamp_){
    _has_odom = false;
    _has_imu = false;
    _offset.setIdentity();
    _imu.setIdentity();
    _odometry.setIdentity();
  }
  
  void BaseSensorMessage::fromStream(std::istream& is) {
    is >> _topic;
    is >> _frame_id;
    is >> _seq;
    is >> _timestamp;
    _has_odom=false;
    _has_imu=false;
    _offset.setIdentity();
    _imu.setIdentity();
    _odometry.setIdentity();
    BaseMessage::fromStream(is);
    Vector6f v;
    for(int i=0;i<v.rows(); i++)      
      is >> v(i);
    _offset = v2t(v);

    is >> _has_odom;
    if (_has_odom) {
      for(int i=0;i<v.rows(); i++)      
	is >> v(i);
      _odometry = v2t(v);
    }
    is >> _has_imu;
    if (_has_imu) {
      for(int i=0;i<v.rows(); i++)      
	is >> v(i);
      _imu = v2t(v);
    }
  }
  
  void  BaseSensorMessage::toStream(std::ostream& os) const {
    BaseMessage::toStream(os);
    char buf[1024];
    sprintf(buf, "%s %s %08d %.5lf", _topic.c_str(), _frame_id.c_str(), _seq, _timestamp);
    os << buf;
    os << " ";
    Vector6f v=t2v(_offset);
    os << v.transpose() << " ";
    os << _has_odom << " ";
    if (_has_odom) {
      v=t2v(_odometry);
      os << v.transpose() << " ";
    }
    os << _has_imu << " ";
    if (_has_imu) {
      v=t2v(_imu);
      os << v.transpose() << " ";
    }
  }



}
