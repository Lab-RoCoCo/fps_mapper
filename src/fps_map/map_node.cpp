#include "boss/eigen_boss_plugin.h"
#include "map_node.h"
#include "GL/gl.h"
#include <iostream>

namespace fps_mapper {
  using namespace std;

  MapNode::MapNode(const Eigen::Isometry3f& t, 
		   int id,
		   boss::IdContext* context):
    boss::Identifiable(id,context){
    _transform = t;
    _timestamp = 0;
  }

  void MapNode::draw(DrawAttributesType attributes, int name) { std::cerr << "MapNode" << std::endl; }  
  
  MapNode::~MapNode() {}

  void MapNode::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
    data.setDouble("timestamp", _timestamp);
    t2v(_transform).toBOSS(data,"transform");

    boss::ArrayData* parentsArray = new boss::ArrayData;
    for (MapNodeSet::iterator it = _parents.begin(); it!=_parents.end(); it++){
      MapNode* node=*it;
      parentsArray->add(new boss::PointerData(node));
    }
    data.setField("parents", parentsArray);
  }

  void MapNode::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
    _timestamp = data.getDouble("timestamp");
    Vector6f v;
    v.fromBOSS(data,"transform");
    _transform = v2t(v);
    boss::ArrayData& parentsArray=data.getField("parents")->getArray();
    _pending_parents.resize(parentsArray.size());
    for (size_t i =0; i< parentsArray.size(); i++){
      boss::ValueData& v = parentsArray[i];
      v.getReference().bind(_pending_parents[i]);
    }
  }
  
  void MapNode::deserializeComplete(){
    _parents.clear();
    for (size_t i =0; i< _pending_parents.size(); i++){
      MapNode* n = dynamic_cast<MapNode*>(_pending_parents[i]);
      _parents.insert(n);
    }
  }
}
