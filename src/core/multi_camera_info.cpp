#include "multi_camera_info.h"
#include <iostream>

namespace fps_mapper {

  using namespace std;

  MultiCameraInfo::MultiCameraInfo(const std::string& topic_,
				   const std::string& frame_id,
				   const Eigen::Isometry3f&offset_,
				   float depth_scale_,
				   int id,
				   boss::IdContext* context):
    BaseCameraInfo(topic_, frame_id, offset_, depth_scale_, id, context){}

  MultiCameraInfo::~MultiCameraInfo() {
    for (std::vector<BaseCameraInfo*>::iterator it=_camera_infos.begin(); it!=_camera_infos.end(); it++){
      delete *it;
    }
  }

  void MultiCameraInfo::serialize(boss::ObjectData& data, boss::IdContext& context) {
    BaseCameraInfo::serialize(data,context);
    boss::ArrayData* camera_info_array = new boss::ArrayData;
    for (std::vector<BaseCameraInfo*>::iterator it = _camera_infos.begin(); it!=_camera_infos.end(); it++){
      BaseCameraInfo* cam=*it;
      camera_info_array->add(new boss::PointerData(cam));
    }
    data.setField("cameraInfos", camera_info_array);
  }

  void MultiCameraInfo::deserialize(boss::ObjectData& data, boss::IdContext& context){
    BaseCameraInfo::deserialize(data,context);
    boss::ArrayData& camera_info_array=data.getField("cameraInfos")->getArray();
    _pending_cameras.resize(camera_info_array.size());
    cerr << "deserialize, offset: " << t2v(_offset).transpose() << endl;;
    for(size_t i = 0; i<camera_info_array.size(); i++){
      boss::ValueData& v = camera_info_array[i];
      v.getReference().bind(_pending_cameras[i]);
    }
  }
    
  void MultiCameraInfo::deserializeComplete() {
    _camera_infos.clear();
    for (size_t i =0; i< _pending_cameras.size(); i++){
      BaseCameraInfo* n = dynamic_cast<BaseCameraInfo*>(_pending_cameras[i]);
      _camera_infos.push_back(n);
    }

  }


  BaseCameraInfo* MultiCameraInfo::scale(float s) {
    MultiCameraInfo* shrunk_cam=new MultiCameraInfo(_topic,
						    _frame_id,
						    _offset,
						    _depth_scale);
    for (size_t i =0; i< _camera_infos.size(); i++){
      BaseCameraInfo* c = _camera_infos[i]->scale(s);
      shrunk_cam->_camera_infos.push_back(c);
    }
    return shrunk_cam;
  }

  BOSS_REGISTER_CLASS(MultiCameraInfo);

}


