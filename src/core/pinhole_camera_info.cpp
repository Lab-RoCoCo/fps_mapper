#include "pinhole_camera_info.h"

namespace fps_mapper {

  using namespace std;
 
  PinholeCameraInfo::PinholeCameraInfo(const std::string& topic_,
				       const std::string& frame_id,
			 const Eigen::Matrix3f& camera_matrix,
			 const Eigen::Isometry3f&offset_,
			 float depth_scale_,
			 int id,
			 boss::IdContext* context):
    BaseCameraInfo(topic_, frame_id, offset_, depth_scale_, id,context){
    _camera_matrix = camera_matrix;
  }

  BaseCameraInfo* PinholeCameraInfo::scale(float s) {
    PinholeCameraInfo* shrunk_cam=new PinholeCameraInfo(_topic,
							_frame_id,
							_camera_matrix,
							_offset,
							_depth_scale);
    shrunk_cam->_camera_matrix.block<2,3>(0,0)*=s;
    return shrunk_cam;
  }
    
  void PinholeCameraInfo::serialize(boss::ObjectData& data, boss::IdContext& context){
    BaseCameraInfo::serialize(data,context);
    _camera_matrix.toBOSS(data, "camera_matrix");
  }

  void PinholeCameraInfo::deserialize(boss::ObjectData& data, boss::IdContext& context){
    BaseCameraInfo::deserialize(data,context);
    _camera_matrix.fromBOSS(data, "camera_matrix");

  }

  BOSS_REGISTER_CLASS(PinholeCameraInfo);
}


