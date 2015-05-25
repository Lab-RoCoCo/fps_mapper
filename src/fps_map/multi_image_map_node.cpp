#include "multi_image_map_node.h"
#include "GL/gl.h"
#include "gl_helpers/opengl_primitives.h"
#include <iostream>

namespace fps_mapper {
  using namespace std;

  MultiImageMapNode::MultiImageMapNode(const Eigen::Isometry3f& t, 
			     MultiCameraInfo* cam,
			     const std::string& topic_,
			     int seq_,
			     int id,
			     boss::IdContext* context) : MapNode(t, id, context){
    _topic = topic_;
    _camera_info = cam;
    _seq = seq_;
  }


  void MultiImageMapNode::serialize(boss::ObjectData& data, boss::IdContext& context){
    MapNode::serialize(data,context);
    data.setPointer("camera_info", _camera_info);
    data.setString("topic", _topic);
    data.setInt("seq",_seq);
    boss::ArrayData* subimages_array = new boss::ArrayData;
    for (size_t i = 0; i<_subimage_seqs.size(); i++)
      subimages_array->add(_subimage_seqs[i]);
    data.setField("subimages_seq", subimages_array);
  }

  void MultiImageMapNode::deserialize(boss::ObjectData& data, boss::IdContext& context){
    MapNode::deserialize(data,context);
    data.getReference("camera_info").bind(_camera_info);
    _topic = data.getString("topic");
    _seq = data.getInt("seq");
    boss::ArrayData& subimages_array = data.getField("subimages_seq")->getArray();
    _subimage_seqs.resize(subimages_array.size());
    for (size_t i =0; i< subimages_array.size(); i++){
      _subimage_seqs[i] = subimages_array[i].getInt();
    }
  }

  void MultiImageMapNode::draw(DrawAttributesType attributes, int name) {
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    if (name>-1)
      glPushName(name);

    // Ask to Giorgio
    glPushMatrix();
    GLHelpers::glMultMatrix(_transform);
    glScalef(0.1, 0.1, 0.1);
    GLHelpers::drawReferenceSystem();    
    glPopMatrix();

    glPushMatrix();
    Eigen::Isometry3f cameraPose = _transform*_camera_info->offset();
    GLHelpers::glMultMatrix(cameraPose);
    for (size_t i = 0; i< _camera_info->cameraInfos().size(); i++){
      glPushMatrix();
      GLHelpers::glMultMatrix(_camera_info->cameraInfos()[i]->offset());
      GLHelpers::drawPyramidWireframe(/*float pyrH = */0.02, /*float pyrW = */0.01);
      glPopMatrix();
    }
    glPopMatrix();
    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(MultiImageMapNode);

}
