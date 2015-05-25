#pragma once
#include "boss/eigen_boss_plugin.h"
#include <set>
#include "globals/defs.h"
#include <tr1/memory>
#include "gl_helpers/draw_attributes.h"
#include "map_node.h"
namespace fps_mapper {

  class BinaryNodeRelation: public boss::Identifiable {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    BinaryNodeRelation(MapNode* from_=0,
		       MapNode* to_=0,
		       const Eigen::Isometry3f& transform=Eigen::Isometry3f::Identity(), 
		       const Matrix6f& info=Matrix6f::Identity(), 
		       int id=-1,
		       boss::IdContext* context=0);

    inline const Eigen::Isometry3f& transform() const {return _transform;}
    inline void setTransform(const Eigen::Isometry3f& t) {_transform = t;}

    inline const Matrix6f& informationMatrix() const {return _information_matrix;}
    inline void setInformationMatrix(const Matrix6f& info) {_information_matrix = info;}
 
    virtual void draw(DrawAttributesType attributes = ATTRIBUTE_SHOW, int name=-1);

    virtual ~BinaryNodeRelation();

    inline const MapNode* from() const {return _from;}
    inline MapNode* from()  {return _from;}
    inline void setFrom(MapNode* n)  {_from = n;}

    inline const MapNode* to() const {return _to;}
    inline MapNode* to()  {return _to;}
    inline void setTo(MapNode* n)  {_to = n;}
    
    inline const MapNode* parent() const {return _parent;}
    inline MapNode* parent() {return _parent;}
    inline void setParent(MapNode* n)  {_parent = n;}

    
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);

  protected:
    Eigen::Isometry3f _transform;
    Matrix6f _information_matrix;
    MapNode* _from;
    MapNode* _to;
    MapNode* _parent;
  };

  typedef std::set< std::tr1::shared_ptr<BinaryNodeRelation> > BinaryNodeRelationSet;
}
