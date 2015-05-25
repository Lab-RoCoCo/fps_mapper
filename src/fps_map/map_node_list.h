#pragma once
#include "boss/eigen_boss_plugin.h"
#include "globals/defs.h"
#include <tr1/memory>
#include "gl_helpers/draw_attributes.h"
#include "map_node.h"
namespace fps_mapper {

  class MapNodeList: public std::list<std::tr1::shared_ptr<MapNode> > {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapNodeList();
    virtual void draw(DrawAttributesType attributes = ATTRIBUTE_SHOW, int name=-1);
    void drawBox();
    inline const Eigen::Vector3f& lowerTranslation() const { return _lower_translation; }
    inline const Eigen::Vector3f& upperTranslation() const { return _upper_translation; }
    inline const Eigen::Vector3f& lowerOrientation() const {return _lower_orientation; }
    inline const Eigen::Vector3f& upperOrientation() const {return _upper_orientation; }
    void addElement(MapNode* tnode);
    void clear();
    virtual ~MapNodeList();
    Eigen::Isometry3f meanPose();
    Eigen::Isometry3f middlePose();
  protected:
    void resetBB();
    Eigen::Vector3f _lower_translation;
    Eigen::Vector3f _upper_translation;
    Eigen::Vector3f _lower_orientation;
    Eigen::Vector3f _upper_orientation;
  };
}
