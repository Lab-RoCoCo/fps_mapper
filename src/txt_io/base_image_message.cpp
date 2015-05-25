#include "base_image_message.h"
#include <cstdio>
#include <cstring>
#include "opencv2/opencv.hpp"


namespace txt_io {
  using namespace std;


  BaseImageMessage::BaseImageMessage(const std::string& topic_, const std::string& frame_id, int  seq_, double timestamp_):
    BaseSensorMessage(topic_, frame_id, seq_, timestamp_){
    _depth_scale = 1e-3;
  }

  BaseImageMessage::~BaseImageMessage() {
    release();
  }

  std::string BaseImageMessage::_binaryFilename() const {
    if (! _binary_filename.length()) {
      std::string fn  = _topic;
      std::replace( fn.begin(), fn.end(), '/', '.');
      if (fn[0]=='.')
	fn=fn.substr(1);
      char buf[1024];
      extension();
      sprintf(buf,"%s_%08d.%s",fn.c_str(),_seq,extension().c_str());
      _binary_filename = buf;
    }
    return _binary_filename;
  }

  void BaseImageMessage::fromStream(istream& is) {
    BaseSensorMessage::fromStream(is);
    is >> _depth_scale;
    is >> _binary_full_filename;
  }

  void BaseImageMessage::toStream(ostream& os) const {
    BaseSensorMessage::toStream(os);
    os << " " << _depth_scale << " ";
    os << binaryFullFilename();
  }

  void BaseImageMessage::_fetch() {
    std::string full_filename = binaryFullFilename();
    _image = cv::imread(full_filename.c_str(), CV_LOAD_IMAGE_UNCHANGED|CV_LOAD_IMAGE_ANYDEPTH);
  }
  
  void BaseImageMessage::_release(){
    _image.release();
  }

  void BaseImageMessage::_writeBack(){
    std::string full_filename = binaryFullFilename();
    cv::imwrite(full_filename.c_str(), _image);
    untaint();
  }
  
  const std::string BaseImageMessage::_tag="BASE_IMAGE_MESSAGE";

  const std::string& BaseImageMessage::tag() const { return _tag; }

  std::string BaseImageMessage::extension() const {
    if (_image.type()==CV_16UC1||_image.type()==CV_8UC1) {
       return "pgm";
    } else if (_image.type()==CV_8UC3) {
      return  "png";
    } else {
      cerr << " image type: " << _image.type() << "rows x cols: " << _image.rows << "x" << _image.cols << endl;
      throw std::runtime_error("Unknown extension for image of this type");
    }
  }
}
