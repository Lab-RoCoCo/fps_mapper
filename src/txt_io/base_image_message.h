#pragma once
#include <string>
#include <iostream>
#include <opencv/cv.h>
#include "base_sensor_message.h"
namespace txt_io {


  class BaseImageMessage: public BaseSensorMessage {
  public:
    BaseImageMessage(const std::string& topic="", const std::string& frame_id="", int seq=-1, double timestamp=-1);
    virtual ~BaseImageMessage();
    inline const cv::Mat& image()  {fetch(); return _image;}
    inline void setImage(cv::Mat& image_) {_image = image_; _is_fetched=true; taint();}
    inline void setDepthScale(float depth_scale) {_depth_scale = depth_scale;}
    inline float depthScale() const { return _depth_scale; }
    virtual const std::string& tag() const;
    virtual void fromStream(std::istream& is);
    virtual void toStream(std::ostream& os) const;
  protected:
    static const std::string _tag;
    virtual void _fetch();
    virtual void _release();
    virtual void _writeBack();
    virtual std::string _binaryFilename() const;
    virtual std::string extension() const;
    float _depth_scale;
    mutable std::string _binary_filename;
    cv::Mat _image;
  };
}
