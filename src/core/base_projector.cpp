#include "base_projector.h"
#include <stdexcept>
#include <omp.h>

namespace fps_mapper {
  using namespace std;


  inline void computeSimpleNormals(Float3Image& normalImage, const Float3Image& pointImage, int xgap,  int ygap, float max_distance = 0.1) {
    normalImage.create(pointImage.rows, pointImage.cols);
    normalImage = cv::Vec3f(0,0,0);
    max_distance *=max_distance;
    for (int r=ygap; r<normalImage.rows-ygap; r++) {
      for (int c=xgap; c<normalImage.cols-xgap; c++) {
	const cv::Vec3f& _p = pointImage.at<cv::Vec3f>(r-ygap,c-xgap);
	if (_p[2] <0)
	  continue;
	Eigen::Vector3f p(_p[0], _p[1], _p[2]);
	const cv::Vec3f& _px = pointImage.at<cv::Vec3f>(r-ygap,c+xgap);
	if (_px[2] <0)
	  continue;
	Eigen::Vector3f px(_px[0], _px[1], _px[2]);
	const cv::Vec3f& _py = pointImage.at<cv::Vec3f>(r+ygap,c-xgap);
	if (_py[2] <0)
	  continue;
	Eigen::Vector3f py(_py[0], _py[1], _py[2]);
	Eigen::Vector3f dx = px-p;
	Eigen::Vector3f dy = py-p;
	if (dx.squaredNorm()>max_distance ||
	    dy.squaredNorm()>max_distance)
	  continue;

	Eigen::Vector3f n = dy.cross(dx);
	n.normalize();
	normalImage.at<cv::Vec3f>(r,c) = cv::Vec3f(n.x(), n.y(), n.z());
      }
    }
  }

  //TODO optimize bu using integral images.
  inline void normalBlur(Float3Image& dest, const Float3Image& src, int window, int start) {
    dest.create(src.rows, src.cols);
    dest = cv::Vec3f(0,0,0);
    for (int r=start+window; r<dest.rows-start-window; r++) {
      for (int c=start+window; c<dest.cols-start-window; c++) {
	cv::Vec3f n(0,0,0);
	for (int rr = -window; rr<=window; rr++) {
	  for (int cc = -window; cc<=window; cc++) {
	    n += src.at<cv::Vec3f>(r+rr,c+cc);
	  }
	}
	float norm = n.dot(n);
	if (norm>0) {
	  n*=(1./sqrt(norm));
	}
	dest.at<cv::Vec3f>(r,c) = n;
      }
    }
  }

  BaseProjector::BaseProjector(){
    _image_cols=640;
    _image_rows=480;
    _min_distance = 0.4;
    _max_distance = 3;
    _offset.setIdentity();
    _inverse_offset.setIdentity();
    _camera_info = 0;
    setIncidenceAngle(0.4 * M_PI);

    _nb_window = 3;
    _cp_window = 3;
    _cp_max_distance = 0.3;
    _raw_depth_scale = 1e-3;
  }

  void BaseProjector::setImageSize(int r, int c) {
    _image_rows = r;
    _image_cols = c;
  }

  void BaseProjector::setCameraInfo(BaseCameraInfo* camera_info){
    _camera_info = camera_info;
    setOffset(_camera_info->offset());
  }

  void BaseProjector::unproject(Cloud& cloud,
				const RawDepthImage& depth_image,
				const RGBImage& rgb_image) {
    // generate the points applying the inverse depth model
    rgb_image.convertTo(_rgb, CV_8UC3);
    unprojectPoints(depth_image);
    computeNormals(cloud, depth_image);
    cloud.transformInPlace(_offset);
  }

  void BaseProjector::computeNormals(Cloud& cloud, const RawDepthImage& depth_image){
    computeSimpleNormals(_normals, _points, _cp_window, _cp_window, _cp_max_distance);
    normalBlur(_smoothed_normals, _normals, _nb_window, _cp_window);
    
    int rstart = _nb_window+_cp_window;
    int rend = depth_image.rows - _nb_window+_cp_window;

    int cstart = _nb_window+_cp_window;
    int cend = depth_image.cols - _nb_window+_cp_window;

    cloud.resize( (rend-rstart)*(cend-cstart) );
    int k = 0;
    //cerr << "nz:" << nz <<endl;
    for (int r=rstart; r<rend; r++){
      for (int c=cstart; c<cend; c++){
	unsigned short d = depth_image.at<unsigned short>(r,c);
	if (!d)
	  continue;
	const cv::Vec3f& _p = _points.at<cv::Vec3f>(r,c);
	if (_p[2]<0)
	  continue;

	float iz = 1./_p[2];
	const cv::Vec3f& _n = _smoothed_normals.at<cv::Vec3f>(r,c);
	Eigen::Vector3f rgbv(0,0,0);
	if (_rgb.rows && _rgb.cols){
	  const cv::Vec3f &rgb = _rgb.at<cv::Vec3f>(r,c);
	  rgbv = Eigen::Vector3f(rgb[0]/255.0, rgb[1]/255.0, rgb[2]/255.0);
	}
	cloud[k] = RichPoint(Eigen::Vector3f(_p[0], _p[1], _p[2]), Eigen::Vector3f(_n[0], _n[1], _n[2]), iz, rgbv);
	k++;
      }
    }
    cloud.resize(k);
  }

}
