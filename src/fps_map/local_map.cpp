#include "local_map.h"
#include "GL/gl.h"
#include "gl_helpers/opengl_primitives.h"
#include <iostream>

namespace fps_mapper {
  using namespace std;

  LocalMap::LocalMap(const Eigen::Isometry3f& transform, 
                     int id,
                     boss::IdContext* context) : MapNode(transform, id, context){
  }

  LocalMap::~LocalMap(){
    _cloud_ref.set(0);
  }
  
  void LocalMap::serialize(boss::ObjectData& data, boss::IdContext& context) {
    MapNode::serialize(data,context);
    boss::ObjectData * blobData=new boss::ObjectData();
    data.setField("cloud", blobData);
    _cloud_ref.serialize(*blobData,context);

    boss::ArrayData* nodesArray = new boss::ArrayData;
    for (MapNodeList::iterator it = _nodes.begin(); it!=_nodes.end(); it++){
      MapNode* node=it->get();
      nodesArray->add(new boss::PointerData(node));
    }
    data.setField("nodes", nodesArray);

    boss::ArrayData* relationsArray = new boss::ArrayData;
    for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++){
      BinaryNodeRelation* rel=it->get();
      relationsArray->add(new boss::PointerData(rel));
    }
    data.setField("relations", relationsArray);

  }
  
  void LocalMap::deserialize(boss::ObjectData& data, boss::IdContext& context) {
    MapNode::deserialize(data,context);

    // deserialize the cloud
    boss::ObjectData * blobData = static_cast<boss::ObjectData *>(data.getField("cloud"));
    _cloud_ref.deserialize(*blobData,context);


    boss::ArrayData& nodesArray=data.getField("nodes")->getArray();
    for (size_t i =0; i< nodesArray.size(); i++){
      boss::ValueData& v = nodesArray[i];
      boss::Identifiable* id = v.getPointer();
      MapNode* n = dynamic_cast<MapNode*>(id);
      _nodes.addElement(n);
    }

    boss::ArrayData& relationsArray=data.getField("relations")->getArray();
    for (size_t i =0; i< relationsArray.size(); i++){
      boss::ValueData& v = relationsArray[i];
      boss::Identifiable* id = v.getPointer();
      BinaryNodeRelation* r = dynamic_cast<BinaryNodeRelation*>(id);
    }
  }


  void LocalMap::draw(DrawAttributesType attributes, int name){
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    
    if (name>-1)
      glPushName(name);

    glPushMatrix();
    GLHelpers::glMultMatrix(_transform);
    
    // draw the cloud, if the local map is selected
    if (attributes&ATTRIBUTE_SELECTED) {
      cloud()->draw(attributes);
      if( ! (attributes&ATTRIBUTE_HIDE_REF) ) {
        _nodes.draw(attributes);
        for (BinaryNodeRelationSet::iterator it = _relations.begin(); it!=_relations.end(); it++) {
          (*it)->draw();
        }
      }
    }

    // draw the reference system coordinates, unless explicitly hidden
    if( ! (attributes&ATTRIBUTE_HIDE_REF) ) {
      glPushMatrix();
      glScalef(0.2, 0.2, 0.2);
      glPushAttrib(GL_COLOR);
      GLHelpers::drawReferenceSystem();

      glPopAttrib();  // GL_COLOR
      glPopMatrix();
    }

    glPopMatrix();

    if (name>-1)
      glPopName();
  }


  BOSS_REGISTER_CLASS(LocalMap);
  
}
