#include "scene_cloud_view.h"
#include "color_handler.h"

namespace pcl {
  namespace gpu {
    namespace kinfuLS {
      void paint3DView(const KinfuTracker::View& rgb24, 
 		       KinfuTracker::View& view, float colors_weight = 0.5f);
      void mergePointNormal(const pcl::gpu::DeviceArray<PointXYZ>& cloud, 
			    const pcl::gpu::DeviceArray<Normal>& normals, 
			    pcl::gpu::DeviceArray<PointNormal>& output);
    }
  }
}

namespace fps_mapper {

  SceneCloudView::SceneCloudView(): 
    _extraction_mode(GPU_Connected6), _compute_normals(false), _valid_combined(false), 
    _cube_added(false), _cloud_viewer("Scene Cloud Viewer") {
    _cloud_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    _normals_ptr = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);
    _combined_ptr = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    _point_colors_ptr = pcl::PointCloud<pcl::RGB>::Ptr(new pcl::PointCloud<pcl::RGB>);
    
    _cloud_viewer.setBackgroundColor(0, 0, 0);
    _cloud_viewer.addCoordinateSystem(1.0, "global");
    _cloud_viewer.initCameraParameters();
    _cloud_viewer.setPosition(0, 500);
    _cloud_viewer.setSize(320, 240);
    _cloud_viewer.setCameraClipDistances (0.01, 10.01);
      
    _cloud_viewer.addText("H: print help", 2, 15, 20, 34, 135, 246);         
    _cloud_viewer.addText("ICP State: ", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
    _cloud_viewer.addText("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
    _cloud_viewer.addText("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");    
  }

  void SceneCloudView::drawCamera(Eigen::Affine3f& pose, const std::string& name, double r, double g, double b) {
    double focal = 575;
    double height = 240;
    double width = 320;
    
    // create a 5-point visual for each camera
    pcl::PointXYZ p1, p2, p3, p4, p5;
    p1.x = 0; p1.y = 0; p1.z = 0;
    double angleX = RAD2DEG (2.0 * atan (width / (2.0 * focal)));
    double angleY = RAD2DEG (2.0 * atan (height / (2.0 * focal)));
    double dist = 0.75;
    double minX, minY, maxX, maxY;
    maxX = dist * tan (atan (width / (2.0 * focal)));
    minX = -maxX;
    maxY = dist * tan (atan (height / (2.0 * focal)));
    minY = -maxY;
    p2.x = minX; p2.y = minY; p2.z = dist;
    p3.x = maxX; p3.y = minY; p3.z = dist;
    p4.x = maxX; p4.y = maxY; p4.z = dist;
    p5.x = minX; p5.y = maxY; p5.z = dist;
    p1 = pcl::transformPoint(p1, pose);
    p2 = pcl::transformPoint(p2, pose);
    p3 = pcl::transformPoint(p3, pose);
    p4 = pcl::transformPoint(p4, pose);
    p5 = pcl::transformPoint(p5, pose);
    std::stringstream ss;
    ss.str("");
    ss << name << "_line1";
    _cloud_viewer.addLine(p1, p2, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line2";
    _cloud_viewer.addLine(p1, p3, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line3";
    _cloud_viewer.addLine(p1, p4, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line4";
    _cloud_viewer.addLine(p1, p5, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line5";
    _cloud_viewer.addLine(p2, p5, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line6";
    _cloud_viewer.addLine(p5, p4, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line7";
    _cloud_viewer.addLine(p4, p3, r, g, b, ss.str());
    ss.str("");
    ss << name << "_line8";
    _cloud_viewer.addLine(p3, p2, r, g, b, ss.str());    
  }

  void SceneCloudView::removeCamera(const std::string& name) {
    _cloud_viewer.removeShape(name);
    std::stringstream ss;
    ss.str("");
    ss << name << "_line1";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line2";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line3";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line4";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line5";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line6";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line7";
    _cloud_viewer.removeShape(ss.str ());
    ss.str("");
    ss << name << "_line8";
    _cloud_viewer.removeShape(ss.str ());
  }

  void SceneCloudView::displayICPState(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool was_lost_) {
    std::string name = "last_good_track";
    std::string name_estimate = "last_good_estimate";
    if(was_lost_ && !kinfu->icpIsLost()) {
      removeCamera(name);
      removeCamera(name_estimate);
      clearClouds(false);
      _cloud_viewer.updateText("ICP State: OK", 450, 55, 20, 0.0, 1.0, 0.0, "icp");
      _cloud_viewer.updateText("Press 'S' to save the current world", 450, 35, 10, 0.0, 1.0, 0.0, "icp_save");
      _cloud_viewer.updateText("Press 'R' to reset the system", 450, 15, 10, 0.0, 1.0, 0.0, "icp_reset");
    }
    else if(!was_lost_ && kinfu->icpIsLost()) { 
      Eigen::Affine3f last_pose = kinfu->getCameraPose();
      drawCamera(last_pose, name, 0.0, 1.0, 0.0);
      _cloud_viewer.updateText("ICP State: LOST", 450, 55, 20, 1.0, 0.0, 0.0, "icp");
      _cloud_viewer.updateText("Press 'S' to save the current world", 450, 35, 10, 1.0, 0.0, 0.0, "icp_save");
      _cloud_viewer.updateText("Press 'R' to reset the system", 450, 15, 10, 1.0, 0.0, 0.0, "icp_reset");
    }        
    if(kinfu->icpIsLost()) {
      removeCamera(name_estimate);
      Eigen::Affine3f last_pose_estimate = kinfu->getLastEstimatedPose();
      drawCamera(last_pose_estimate, name_estimate, 1.0, 0.0, 0.0);      
    }
  }

   void SceneCloudView::show(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool integrate_colors) {
      _viewer_pose = kinfu->getCameraPose();

      ScopeTimeT time ("PointCloud Extraction");
      std::cout << "\n[INFO]: getting cloud... " << std::flush;

      _valid_combined = false;

      if(_extraction_mode != GPU_Connected6) { kinfu->volume().fetchCloudHost (*_cloud_ptr, _extraction_mode == CPU_Connected26); }
      else {
	pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu->volume().fetchCloud (_cloud_buffer_device);             
	  if(_compute_normals) {
	    kinfu->volume().fetchNormals(extracted, _normals_device);
	    pcl::gpu::kinfuLS::mergePointNormal(extracted, _normals_device, _combined_device);
	    _combined_device.download(_combined_ptr->points);
	    _combined_ptr->width = (int)_combined_ptr->points.size();
	    _combined_ptr->height = 1;	    
	    _valid_combined = true;
	  }
	  else {
	    extracted.download(_cloud_ptr->points);
	    _cloud_ptr->width = (int)_cloud_ptr->points.size();
	    _cloud_ptr->height = 1;
	  }
	  if(integrate_colors) {
	    kinfu->colorVolume().fetchColors(extracted, _point_colors_device);
	    _point_colors_device.download(_point_colors_ptr->points);
	    _point_colors_ptr->width = (int)_point_colors_ptr->points.size();
	    _point_colors_ptr->height = 1;
	  }
	  else { _point_colors_ptr->points.clear(); }
	}
      size_t points_size = _valid_combined ? _combined_ptr->points.size() : _cloud_ptr->points.size();
      cout << " done. Cloud size: " << points_size / 1000 << "K" << endl;

      _cloud_viewer.removeAllPointClouds();    
      if(_valid_combined) {
	pcl::visualization::PointCloudColorHandlerRGBHack<pcl::PointNormal> rgb(_combined_ptr, _point_colors_ptr);
	_cloud_viewer.addPointCloud<pcl::PointNormal>(_combined_ptr, rgb, "Cloud");
	_cloud_viewer.addPointCloudNormals<pcl::PointNormal>(_combined_ptr, 50);
      }
      else {
	pcl::visualization::PointCloudColorHandlerRGBHack<pcl::PointXYZ> rgb(_cloud_ptr, _point_colors_ptr);
	_cloud_viewer.addPointCloud<pcl::PointXYZ>(_cloud_ptr, rgb);
      }    
   }

  void SceneCloudView::toggleCube(const Eigen::Vector3f& size) {
    if(_cube_added) { _cloud_viewer.removeShape("cube"); }
    else { _cloud_viewer.addCube(size * 0.5, Eigen::Quaternionf::Identity(), size(0), size(1), size(2)); }
    _cube_added = !_cube_added;
  }

   void SceneCloudView::toggleExtractionMode() {
     _extraction_mode = (_extraction_mode + 1) % 3;
     switch(_extraction_mode) {
     case 0: std::cout << "[INFO]: cloud extraction mode: GPU, Connected-6" << std::endl; break;
     case 1: std::cout << "[INFO]: cloud extraction mode: CPU, Connected-6    (requires a lot of memory)" << std::endl; break;
     case 2: std::cout << "[INFO]: cloud extraction mode: CPU, Connected-26   (requires a lot of memory)" << std::endl; break;
     }         
   }

  void SceneCloudView::toggleNormals() {
    _compute_normals = !_compute_normals;
    std::cout << "[INFO]: compute normals: " << (_compute_normals ? "On" : "Off") << std::endl;
  }

  void SceneCloudView::clearClouds(bool print_message) {
    _cloud_viewer.removeAllPointClouds();
    _cloud_ptr->points.clear();
    _normals_ptr->points.clear();    
    if(print_message) { std::cout << "[INFO]: clouds/meshes were cleared" << std::endl; }
  }
  
  void SceneCloudView::showMesh(pcl::gpu::kinfuLS::KinfuTracker* kinfu, bool /*integrate_colors*/) {
    ScopeTimeT time ("Mesh Extraction");
    std::cout << "\n[INFO]: getting mesh... " << std::flush;
    
    if(!_marching_cubes) { _marching_cubes = pcl::gpu::kinfuLS::MarchingCubes::Ptr(new pcl::gpu::kinfuLS::MarchingCubes()); }
    
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device = _marching_cubes->run(kinfu->volume(), _triangles_buffer_device);    
    _mesh_ptr = convertToMesh(triangles_device);
    
    _cloud_viewer.removeAllPointClouds();
    if(_mesh_ptr) { _cloud_viewer.addPolygonMesh(*_mesh_ptr); }
    
    std::cout << " done. Triangles number: " << triangles_device.size() / pcl::gpu::kinfuLS::MarchingCubes::POINTS_PER_TRIANGLE / 1000 << "K" << std::endl;
  }

  boost::shared_ptr<pcl::PolygonMesh> SceneCloudView::convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles) { 
    if(triangles.empty()) { return boost::shared_ptr<pcl::PolygonMesh>(); }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width  = (int)triangles.size();
    cloud.height = 1;
    triangles.download(cloud.points);

    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr(new pcl::PolygonMesh()); 
    pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);

    mesh_ptr->polygons.resize(triangles.size() / 3);
    for(size_t i = 0; i < mesh_ptr->polygons.size(); ++i) {
      pcl::Vertices v;
      v.vertices.push_back(i * 3 + 0);
      v.vertices.push_back(i * 3 + 2);
      v.vertices.push_back(i * 3 + 1);              
      mesh_ptr->polygons[i] = v;
    }    
    return mesh_ptr;
  }

  Eigen::Affine3f SceneCloudView::getViewerPose(pcl::visualization::PCLVisualizer& viewer) {
    Eigen::Affine3f pose = viewer.getViewerPose();
    Eigen::Matrix3f rotation = pose.linear();
    
    Eigen::Matrix3f axis_reorder;  
    axis_reorder << 
       0,  0,  1,
      -1,  0,  0,
       0, -1,  0;
    
    rotation = rotation * axis_reorder;
    pose.linear() = rotation;
    return pose;
  }

  void SceneCloudView::setViewerPose(pcl::visualization::PCLVisualizer& viewer, 
				     const Eigen::Affine3f& viewer_pose) {
    Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
    Eigen::Vector3f look_at_vector = viewer_pose.rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
    Eigen::Vector3f up_vector = viewer_pose.rotation() * Eigen::Vector3f(0, -1, 0);
    viewer.setCameraPosition(pos_vector[0], pos_vector[1], pos_vector[2],
			     look_at_vector[0], look_at_vector[1], look_at_vector[2],
			     up_vector[0], up_vector[1], up_vector[2]);
  }

}
