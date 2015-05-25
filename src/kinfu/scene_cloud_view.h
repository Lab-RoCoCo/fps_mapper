#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/common/time.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fps_mapper {

  typedef pcl::ScopeTime ScopeTimeT;


  class SceneCloudView {
  public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum { GPU_Connected6 = 0, CPU_Connected6 = 1, CPU_Connected26 = 2 };

    SceneCloudView();
  
    void drawCamera(Eigen::Affine3f& pose, const std::string& name, double r, double g, double b);  
    void removeCamera(const std::string& name);
    void displayICPState(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool was_lost_); 
      
    void show(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool integrate_colors);

    void toggleCube(const Eigen::Vector3f& size);
    void toggleExtractionMode();
    void toggleNormals();
    
    void clearClouds(bool print_message = false);
    void showMesh(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool /*integrate_colors*/);
      
    boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles);
    Eigen::Affine3f getViewerPose(pcl::visualization::PCLVisualizer& viewer);
    void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

    pcl::visualization::PCLVisualizer& cloudViewer() { return _cloud_viewer; };

  protected:
    int _extraction_mode;
    bool _compute_normals;
    bool _valid_combined;
    bool _cube_added;

    Eigen::Affine3f _viewer_pose;

    pcl::visualization::PCLVisualizer _cloud_viewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr _cloud_ptr;
    pcl::PointCloud<pcl::Normal>::Ptr _normals_ptr;

    pcl::gpu::DeviceArray<pcl::PointXYZ> _cloud_buffer_device;
    pcl::gpu::DeviceArray<pcl::Normal> _normals_device;

    pcl::PointCloud<pcl::PointNormal>::Ptr _combined_ptr;
    pcl::gpu::DeviceArray<pcl::PointNormal> _combined_device;  

    pcl::gpu::DeviceArray<pcl::RGB> _point_colors_device; 
    pcl::PointCloud<pcl::RGB>::Ptr _point_colors_ptr;

    pcl::gpu::kinfuLS::MarchingCubes::Ptr _marching_cubes;
    pcl::gpu::DeviceArray<pcl::PointXYZ> _triangles_buffer_device;

    boost::shared_ptr<pcl::PolygonMesh> _mesh_ptr;
  };

}
