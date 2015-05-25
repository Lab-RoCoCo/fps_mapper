#pragma once

#include "gl_helpers/simple_viewer.h"
#include "fps_map/map_node_list.h"
#include "fps_map/binary_node_relation.h"

namespace fps_mapper {

  class TrajectoryViewer: public GLHelpers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapNodeList nodes;
    BinaryNodeRelationSet relations;
    virtual void draw();
    virtual void drawWithNames();
  protected:
    std::map<int, MapNode*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<MapNode*> _selected_objects;
  };
 
}

