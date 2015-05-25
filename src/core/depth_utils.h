#pragma once

#include "globals/defs.h"
#include "cloud.h"

namespace fps_mapper {

  void shrinkRawDepth(RawDepthImage& dest_buffer, const RawDepthImage& src_buffer, int k);

  void shrinkDepth(FloatImage& dest_buffer, IntImage& dest_indices, 
		    const FloatImage& src_buffer, const IntImage& src_indices, int k);

  void compareDepths(float& in_distance, 
		     int& in_num,
		     float& out_distance, 
		     int& out_num,
		     const FloatImage& depths1, const IntImage& indices1, 
		     const FloatImage& depths2, const IntImage& indices2, 
		     float dist=0.05, bool scale_z = false, FloatImage* result=0);

  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, float scale = 1000.0f);
  void convert_32FC1_to_16UC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 1000.0f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, float scale = 0.001f);
  void convert_16UC1_to_32FC1(cv::Mat& dest, const cv::Mat& src, const cv::Mat& mask, float scale = 0.001f);

}
