#include "multi_projector.h"
#include "multi_camera_info.h"
#include "pinhole_camera_info.h"
#include "pinhole_projector.h"
#include <stdexcept>

namespace fps_mapper {
  MultiProjector::MultiProjector(){
    _mono_rows = 0;
  }

  MultiProjector::MultiProjector(std::vector<BaseProjector*>& projectors_):
    _projectors(projectors_){
    _mono_rows = 0;
  }

  MultiProjector::~MultiProjector(){
    for (size_t i = 0; i<_projectors.size();i++)
      delete _projectors[i];
  }

  void MultiProjector::setImageSize(int r, int c) {
    BaseProjector::setImageSize(r,c);
    _mono_rows = r/_projectors.size();
    if (r%_projectors.size()){
      cerr << "rows " << r << endl;
      cerr << "cols " << c << endl;
      cerr << "# proj: " << _projectors.size() << endl;
      throw std::runtime_error("the # of projectors should divide the columnsperfectly"); 
    }
    
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (_projectors[i])
	_projectors[i]->setImageSize(_mono_rows, c);
    }
  }

  void MultiProjector::initFromCameraInfo(BaseCameraInfo* _camera_info){
    MultiCameraInfo* mci  = dynamic_cast<MultiCameraInfo*>(_camera_info);
    if (! mci) {
      throw std::runtime_error("Multi Projector needs a Multi CameraInfo");
    }
    _projectors.clear();
    _projectors.resize(mci->cameraInfos().size());
    for (size_t i = 0; i<mci->cameraInfos().size(); i++){
      PinholeCameraInfo * pci =dynamic_cast<PinholeCameraInfo*>(mci->cameraInfos()[i]);
      if (pci){
	PinholeProjector* pp= new PinholeProjector;
	_projectors[i] = pp;
	pp->setMaxDistance(_max_distance);
	pp->setMinDistance(_min_distance);
	pp->setIncidenceAngle(_incidence_angle);
      } else 
	throw std::runtime_error("unknown projector type");
    }
    setCameraInfo(mci);
  }

  void MultiProjector::setCameraInfo(BaseCameraInfo* camera_info) {
    MultiCameraInfo* mci  = dynamic_cast<MultiCameraInfo*>(camera_info);
    if (! mci) {
      throw std::runtime_error("Multi Projector needs a Multi CameraInfo");
    }
    _camera_info = mci;
    
    if (mci->cameraInfos().size()!=_projectors.size())
      throw std::runtime_error("num of projectors does not match");

    for (size_t i=0; i<mci->cameraInfos().size(); i++){
      if (_projectors[i])
	_projectors[i]->setCameraInfo(mci->cameraInfos()[i]);
    }
    setOffset(mci->offset());

  }

  void MultiProjector::project(FloatImage& zbuffer, 
			       IndexImage& indices,
			       const Eigen::Isometry3f& T,
			       const Cloud& model) const {
    zbuffer.create(_image_rows, _image_cols);
    indices.create(_image_rows, _image_cols);

    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;

      float* zb= zbuffer.ptr<float>(_mono_rows*i);
      int* ib= indices.ptr<int>(_mono_rows*i);
      FloatImage mono_zbuffer(_mono_rows, _image_cols,zb); 
      IntImage mono_indices(_mono_rows, _image_cols,ib); 
      _projectors[i]->project(mono_zbuffer, mono_indices, _inverse_offset*T,model);
    }    
  }

    // Throws and exception
  void MultiProjector::project(FloatImage& zbuffer, 
			 IndexImage& indices,
			 const Eigen::Isometry3f& T,
			 const FloatImage& src_zbuffer, 
			 const IntImage& src_indices,
			 float src_scale,
			 int subsample) const {
    throw std::runtime_error("not implemented");
  }

  void MultiProjector::unproject(Cloud& cloud, const RawDepthImage& depth_image, const RGBImage& rgb_image){
    Cloud temp;
    cloud.clear();
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i]) {
	continue;
      }
      unsigned short * zb= const_cast<unsigned short*>(depth_image.ptr<const unsigned short>(_mono_rows*i));


      RGBImage mono_rgb;
      RawDepthImage mono_zbuffer(_mono_rows, _image_cols,zb); 
      if (rgb_image.rows && rgb_image.cols) {
	cv::Vec3b * rgbb= const_cast<cv::Vec3b*>(rgb_image.ptr<const cv::Vec3b>(_mono_rows*i));
	mono_rgb = RGBImage(_mono_rows, _image_cols, rgbb);
      }
      _projectors[i]->unproject(temp, mono_zbuffer, mono_rgb);
      cloud.add(temp);
    }
    cloud.transformInPlace(_camera_info->offset());
  }


  void MultiProjector::unprojectPoints(const RawDepthImage& depth_image){}

  void MultiProjector::setMaxDistance(float d) {
    _max_distance = d;
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;
      _projectors[i]->setMaxDistance(_max_distance);
    }
  }


  void MultiProjector::setMinDistance(float d) {
    _min_distance = d;
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;
      _projectors[i]->setMinDistance(_min_distance);
    }
  }
  
  void MultiProjector::scaleCamera(float s) {
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;
      _projectors[i]->scaleCamera(s);
    }
  }

  MultiProjector::State::State(const Eigen::Isometry3f& offset, int r, int c) {
    this->offset = offset;
    image_rows = r;
    image_cols = c;
  }

  void MultiProjector::pushState(){
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;
      _projectors[i]->pushState();
    }
    State s(offset(), _image_rows, _image_cols);
    _states.push(s);
  }


  void MultiProjector::popState(){
    for (size_t i = 0; i<_projectors.size(); i++) {
      if (!_projectors[i])
	continue;
      _projectors[i]->popState();
    }
    State s = _states.top();
    _image_rows = s.image_rows;
    _image_cols = s.image_cols;
    setOffset(s.offset);
    _states.pop();
  }
  
  
}
