#include "depth_utils.h"
#include <iostream>
#include <omp.h>
#include <stdexcept>
using namespace std;

#define _NAN_CHECK_
namespace fps_mapper {

  void shrinkRawDepth(RawDepthImage& dest_buffer, const RawDepthImage& src_buffer, int k){

    int rows = src_buffer.rows;
    int cols = src_buffer.cols;
    

    if (rows%k) {
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows");
    }

    if (cols%k) {
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols");
    }

    int drows = rows/k;
    int dcols = cols/k;


    dest_buffer.create(drows, dcols);
    dest_buffer=0;

    // avoid divisions and use a lookup table
    int lv = rows>cols?rows:cols;
    int ttable[lv];
    for (int i = 0; i<lv; i++)
      ttable[i] = i/k;

    for (int r = 0; r<rows; r++) {
      // get the row pointers of source and destination

      const unsigned short* src_z_ptr =src_buffer.ptr<unsigned short>(r);
      int dr = ttable[r];
      unsigned short* dest_z_ptr = dest_buffer.ptr<unsigned short>(dr);
      
      int cc = 0;
      for (int c = 0; c<cols; c++){
	unsigned short src_z = *src_z_ptr;
	src_z_ptr++;
	if (src_z==0)
	  continue;
	unsigned short& dest_z = *(dest_z_ptr+ttable[c]);
	if (! dest_z || dest_z<src_z)
	  dest_z = src_z;
      }
    }
  }


  void shrinkDepth(FloatImage& dest_buffer, IntImage& dest_indices, 
		   const FloatImage& src_buffer, const IntImage& src_indices, int k){

    int rows = src_buffer.rows;
    int cols = src_buffer.cols;
    
    if (src_buffer.rows!=src_indices.rows || src_buffer.cols!=src_indices.cols) {
      throw std::runtime_error("shrinkDepth: src indices and buffer do not match");
    }

    if (rows%k) {
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image rows");
    }

    if (cols%k) {
      throw std::runtime_error("shrinkDepth: fatal, the shrink factor should be perfect divider of the image cols");
    }

    int drows = rows/k;
    int dcols = cols/k;


    dest_buffer.create(drows, dcols);
    dest_buffer=1e3f;

    dest_indices.create(drows, dcols);
    dest_indices = -1;

    // avoid divisions and use a lookup table
    int lv = rows>cols?rows:cols;
    int ttable[lv];
    for (int i = 0; i<lv; i++)
      ttable[i] = i/k;

    for (int r = 0; r<rows; r++) {
      // get the row pointers of source and destination

      const float* src_z_ptr =src_buffer.ptr<float>(r);
      const int* src_idx_ptr=src_indices.ptr<int>(r);
      int dr = ttable[r];
      float* dest_z_ptr = dest_buffer.ptr<float>(dr);
      int* dest_idx_ptr = dest_indices.ptr<int>(dr);
      
      int cc = 0;
      for (int c = 0; c<cols; c++){
	float src_z = *src_z_ptr;
	int src_idx = *src_idx_ptr;

	src_z_ptr++;
	src_idx_ptr++;

	if (src_idx<0)
	  continue;
	
	float& dest_z = *(dest_z_ptr+ttable[c]);
	int& dest_idx = *(dest_idx_ptr+ttable[c]);
	
	if (dest_idx<0 || (dest_idx>=0 && dest_z>=src_z)) {
	  dest_idx = src_idx;
	  dest_z = src_z;
	}
      }
    }
  }

  void compareDepths(float& in_distance, 
		     int& in_num,
		     float& out_distance, 
		     int& out_num,
		     const FloatImage& depths1, const IntImage& indices1, 
		     const FloatImage& depths2, const IntImage& indices2, 
		     float dist, bool scale_z, FloatImage* result) {
    if (depths1.rows!=indices1.rows ||
	depths1.cols!=indices1.cols)
      throw std::runtime_error("compareDepths: image1 size mismatch");

    if (depths2.rows!=indices2.rows ||
	depths2.cols!=indices2.cols)
      throw std::runtime_error("compareDepths: image2 size mismatch");

    if (depths1.rows!=depths2.rows ||
	depths1.cols!=depths2.cols)
      throw std::runtime_error("compareDepths: image1 - image2 size mismatch");
    

    in_distance = 0;
    in_num = 0;
    out_distance = 0;
    out_num = 0;
    if (result) {
      result->create(depths1.rows, depths1.cols);
      *result = 6.0f;
    }
    for(int r = 0; r<depths1.rows; r++) {
      const float* d1_ptr = depths1.ptr<float>(r);
      const float* d2_ptr = depths2.ptr<float>(r);
      float* res_ptr = 0;
      if (result) {
	res_ptr=result->ptr<float>(r);
      }
      const int* i1_ptr = indices1.ptr<int>(r);
      const int* i2_ptr = indices2.ptr<int>(r);
      for (int c = 0; c<depths1.cols; c++){
	int i1 = *i1_ptr;
	float d1 = *d1_ptr;
	int i2 = *i2_ptr;
	float d2 = *d2_ptr;
	if (res_ptr)
	  *res_ptr = 6;

	if (i1>-1 && i2>-1) {
	  float avg = .5* (d1+d2);
	  float d = fabs(d1-d2);
	  d=scale_z ? d/avg : d;
	  if (d<dist){
	    if (res_ptr)
	      *res_ptr = 6;
	    in_num ++;
	    in_distance += d;
	  } else {
	    if (res_ptr && d<0.5)
	      *res_ptr = d;
	    out_num ++;
	    out_distance += d;
	  }
	}
	i1_ptr++;
	i2_ptr++;
	d1_ptr++;
	d2_ptr++;
	    if (res_ptr)
	res_ptr++;
      }
    }


  }

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, float scale) {
    assert(src.type() != CV_32FC1 && "convert_32FC1_to_16UC1: source image of different type from 32FC1");
    const float* sptr = (const float*)src.data;
    int size = src.rows * src.cols;
    const float* send = sptr + size;
    dest.create(src.rows, src.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr < send) {
      if(*sptr >= 1e9f) { *dptr = 0; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
    }
  }

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale) {
    assert(src.type() != CV_32FC1 && "convert_32FC1_to_16UC1: source image of different type from 32FC1");
    const int* mptr = (const int*)mask.data;
    const float* sptr = (const float*)src.data;
    int size = src.rows * src.cols;
    const float* send = sptr + size;
    dest.create(src.rows, src.cols, CV_16UC1);
    dest.setTo(cv::Scalar(0));
    unsigned short* dptr = (unsigned short*)dest.data;
    while(sptr < send) {
      if(*mptr < 0) { *dptr = 0; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
      ++mptr;
    }
  }

  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, float scale) {
    assert(src.type() != CV_16UC1 && "convert_16UC1_to_32FC1: source image of different type from 16UC1");
    const unsigned short* sptr = (const unsigned short*)src.data;
    int size = src.rows * src.cols;
    const unsigned short* send = sptr + size;
    dest.create(src.rows, src.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0.0f));
    float* dptr = (float*)dest.data;
    while(sptr < send) {
      if(*sptr == 0) { *dptr = 1e9f; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
    }
  }  

  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale) {
    assert(src.type() != CV_16UC1 && "convert_16UC1_to_32FC1: source image of different type from 16UC1");
    const int* mptr = (const int*)mask.data;
    const unsigned short* sptr = (const unsigned short*)src.data;
    int size = src.rows * src.cols;
    const unsigned short* send = sptr + size;
    dest.create(src.rows, src.cols, CV_32FC1);
    dest.setTo(cv::Scalar(0.0f));
    float* dptr = (float*)dest.data;
    while(sptr < send) {
      if(*mptr < 0) { *dptr = 1e9f; }
      else { *dptr = scale * (*sptr); }
      ++dptr;
      ++sptr;
      ++mptr;
    }
  }  
  
}
