#pragma once
#include "base_camera_info.h"

namespace fps_mapper {

  class CameraInfoManager: public boss::Identifiable {
  public:
    CameraInfoManager(int id=-1,
		      boss::IdContext* context=0);

    ~CameraInfoManager() ;

    BaseCameraInfo* getCamera(const std::string& topic) ;

    BaseCameraInfo* hasCamera(BaseCameraInfo* cam);
    
    void addCamera(BaseCameraInfo* cam);

    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    
    std::vector<BaseCameraInfo*>& cameras() {return _camera_info_vector; }
  protected:
    std::vector<BaseCameraInfo*> _camera_info_vector;
    std::map<std::string, BaseCameraInfo*> _camera_info_map;
    std::set<BaseCameraInfo*> _camera_info_set;
  };

}
