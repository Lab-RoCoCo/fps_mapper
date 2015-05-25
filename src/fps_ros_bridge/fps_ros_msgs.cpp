#include "fps_ros_msgs.h"
#include <iostream>
using namespace std;


namespace fps_mapper {

  void cloud2msg(fps_mapper::CloudMsg& dest, const fps_mapper::Cloud& src) {
    dest.points.resize(src.size());
    for (size_t i = 0; i<src.size(); i++) {
      fps_mapper::RichPoint p = src[i];
      p.normalize();
      dest.points[i].point.x = p.point().x();
      dest.points[i].point.y = p.point().y();
      dest.points[i].point.z = p.point().z();
      dest.points[i].normal.x = p.normal().x();
      dest.points[i].normal.y = p.normal().y();
      dest.points[i].normal.z = p.normal().z();
      dest.points[i].rgb.x = p.rgb().x();
      dest.points[i].rgb.y = p.rgb().y();
      dest.points[i].rgb.z = p.rgb().z();
      dest.points[i].accumulator = p.accumulator();
    }
  }

  void msg2cloud(fps_mapper::Cloud& dest,    const fps_mapper::CloudMsg& src) {
    dest.resize(src.points.size());
    for (size_t i = 0; i<src.points.size(); i++) {
      const fps_mapper::RichPointMsg& p = src.points[i];
      dest[i] = fps_mapper::RichPoint( Eigen::Vector3f(p.point.x, p.point.y, p.point.z),
				       Eigen::Vector3f(p.normal.x, p.normal.y, p.normal.z),
				       p.accumulator,
				       Eigen::Vector3f(p.rgb.x, p.rgb.y, p.rgb.z));
    }
  }


  PinholeCameraInfo* msg2pinholeCameraInfo(const PinholeCameraInfoMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    // cerr<< "cam, id: " << id << endl;
    PinholeCameraInfo* cam;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      cam = dynamic_cast<PinholeCameraInfo*>(o);
    } else {
      cam = new PinholeCameraInfo;
      cam->setId(id);
      context->add(cam);
    } 
    cam->setOffset(pose2eigen(msg.offset));
    cam->setTopic(msg.topic);
    cam->setDepthScale(msg.depth_scale);
    Eigen::Matrix3f cmat;
    int k = 0; 
    for(int r = 0; r<3; r++)
      for(int c = 0; c<3; c++)
	cmat(r,c)=msg.camera_matrix[k++];
    cam->setCameraMatrix(cmat);
    return cam;
  }

  PinholeCameraInfoMsg pinholeCameraInfo2msg(PinholeCameraInfo* src, boss::IdContext*  context) {
    PinholeCameraInfoMsg msg;
    if (src->getId()==-1)
      src->setId(context->generateId());
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.offset = eigen2pose(src->offset());
    msg.topic = src->topic();
    msg.frame_id = src->frameId();
    int k = 0; 
    for(int r = 0; r<3; r++)
      for(int c = 0; c<3; c++) {
	msg.camera_matrix[k] = src->cameraMatrix()(r,c);
	k++;
      }
    return msg;
  }
 

  MultiCameraInfo* msg2multiCameraInfo(const MultiCameraInfoMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    // cerr<< "mcam, id: " << id << endl;
    MultiCameraInfo* cam;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      cam = dynamic_cast<MultiCameraInfo*>(o);
    } else {
      cam = new MultiCameraInfo;
      cam->setId(id);
      context->add(cam);
    } 
    cam->setOffset(pose2eigen(msg.offset));
    cam->setTopic(msg.topic);
    cam->setDepthScale(msg.depth_scale);
    cam->cameraInfos().resize(msg.subcamera_ids.size());
    for(size_t i = 0; i< msg.subcamera_ids.size(); i++){
      boss::Identifiable* o=context->getById(msg.subcamera_ids[i]);
      if (! o ) {
	return 0;
      }
      BaseCameraInfo* cinfo=dynamic_cast<BaseCameraInfo*>(o);
      if (! cinfo)
	throw std::runtime_error("wrong object type");
      cam->cameraInfos()[i]=cinfo;
    }
    return cam;
  }

  MultiCameraInfoMsg multiCameraInfo2msg(MultiCameraInfo* src, boss::IdContext*  context) {
    MultiCameraInfoMsg msg;
    if (src->getId()==-1)
      src->setId(context->generateId());
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.offset = eigen2pose(src->offset());
    msg.topic = src->topic();
    msg.frame_id = src->frameId();
    msg.subcamera_ids.resize(src->cameraInfos().size());
    for (size_t i = 0; i<src->cameraInfos().size(); i++){
      BaseCameraInfo* cam = src->cameraInfos()[i];
      if (cam->getId()==-1)
	cam->setId(context->generateId());
      cam->setContext(context);
      context->add(cam);
      msg.subcamera_ids[i] = cam->getId();
    }
    return msg;
  }

  MapNode* msg2MapNode(const MapNodeMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    
    MapNode* node;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      node = dynamic_cast<MapNode*>(o);
    } else {
      node = new MapNode;
      node->setId(id);
      context->add(node);
    } 
    node->setTransform(pose2eigen(msg.transform));
    node->setTimestamp(msg.timestamp);
    /*
    for(size_t i = 0; i< msg.parents.size(); i++){
      boss::Identifiable* o=context->getById(msg.parents[i]);
      if (! o ) {
	throw std::runtime_error("invalid object instantiated");
      }
      MapNode* n=dynamic_cast<MapNode*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      node->parents().insert(n);
    }
    */
    return node;
  }

  MapNodeMsg mapNode2msg(MapNode* src, boss::IdContext* context) {
    MapNodeMsg msg;
    if (src->getId()==-1)
      src->setId(context->generateId());
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.transform = eigen2pose(src->transform());
    msg.parents.resize(src->parents().size());
    int k=0;
    for (MapNodeSet::iterator it=src->parents().begin(); it!=src->parents().end(); it++){
      MapNode* n = *it;
      if (n->getId()==-1)
	n->setId(context->generateId());
      n->setContext(context);
      context->add(n);
      msg.parents[k] = n->getId();
      k++;
    }
    msg.timestamp = src->timestamp();
    return msg;
  }


  ImageMapNode* msg2imageMapNode(const ImageMapNodeMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    
    ImageMapNode* node;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      node = dynamic_cast<ImageMapNode*>(o);
    } else {
      node = new ImageMapNode;
      node->setId(id);
      context->add(node);
    } 
    node->setTransform(pose2eigen(msg.transform));
    /*
    for(size_t i = 0; i< msg.parents.size(); i++){
      boss::Identifiable* o=context->getById(msg.parents[i]);
      if (! o ) {
	throw std::runtime_error("invalid object instantiated");
      }
      MapNode* n=dynamic_cast<MapNode*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      node->parents().insert(n);
    }
    */
    o = context->getById(msg.camera_info);
    if (! o)
      return 0;
    BaseCameraInfo* cinfo = dynamic_cast<BaseCameraInfo*>(o);
    if (cinfo)
      node->setCameraInfo(cinfo);
    node->setTopic(msg.topic);
    node->setSeq(msg.seq);
    node->setTimestamp(msg.timestamp);

    return node;
  }

  ImageMapNodeMsg imageMapNode2msg(ImageMapNode* src, boss::IdContext* context) {
    ImageMapNodeMsg msg;
    if (src->getId()==-1)
      src->setId(context->generateId());
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.transform = eigen2pose(src->transform());
    msg.parents.resize(src->parents().size());
    int k=0;
    for (MapNodeSet::iterator it=src->parents().begin(); it!=src->parents().end(); it++){
      MapNode* n = *it;
      if (n->getId()==-1)
	n->setId(context->generateId());
      n->setContext(context);
      context->add(n);
      msg.parents[k] = n->getId();
      k++;
    }
    msg.topic = src->topic();
    if (!src->cameraInfo())
      throw std::runtime_error ("camera info not serialized");
    msg.camera_info = src->cameraInfo()->getId();
    msg.seq = src->seq();
    msg.timestamp = src->timestamp();
    return msg;
  }
  
  MultiImageMapNode* msg2multiImageMapNode(const MultiImageMapNodeMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    
    MultiImageMapNode* node;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      node = dynamic_cast<MultiImageMapNode*>(o);
    } else {
      node = new MultiImageMapNode;
      node->setId(id);
      context->add(node);
    } 
    // cerr<< "MMn: " << id << endl;
      
    node->setTransform(pose2eigen(msg.transform));
    /*
    for(size_t i = 0; i< msg.parents.size(); i++){
      boss::Identifiable* o=context->getById(msg.parents[i]);
      if (! o ) {
	throw std::runtime_error("invalid object instantiated");
      }
      MapNode* n=dynamic_cast<MapNode*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      node->parents().insert(n);
    }
    */
    o = context->getById(msg.camera_info);
    if (! o)
      return 0;
    MultiCameraInfo* cinfo = dynamic_cast<MultiCameraInfo*>(o);
    if (cinfo)
      node->setCameraInfo(cinfo);
    node->setTopic(msg.topic);
    node->setSeq(msg.seq);
    node->setTimestamp(msg.timestamp);
    node->subimageSeqs().resize(msg.subimage_seqs.size());
    for(size_t i = 0; i< msg.subimage_seqs.size(); i++){
     node->subimageSeqs()[i] = msg.subimage_seqs[i];
    }
    return node;
  }

  MultiImageMapNodeMsg multiImageMapNode2msg(MultiImageMapNode* src, boss::IdContext* context) {
    MultiImageMapNodeMsg msg;
    if (src->getId()==-1)
      src->setId(context->generateId());
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.transform = eigen2pose(src->transform());
    msg.parents.resize(src->parents().size());
    int k=0;
    for (MapNodeSet::iterator it=src->parents().begin(); it!=src->parents().end(); it++){
      MapNode* n = *it;
      if (n->getId()==-1)
	n->setId(context->generateId());
      n->setContext(context);
      context->add(n);
      msg.parents[k] = n->getId();
      k++;
    }
    msg.topic = src->topic();
    if (!src->cameraInfo())
      throw std::runtime_error ("camera info not serialized");
    msg.camera_info = src->cameraInfo()->getId();
    msg.seq = src->seq();
    msg.subimage_seqs.resize(src->subimageSeqs().size());
    for (size_t i =0; i<src->subimageSeqs().size(); i++)
      msg.subimage_seqs[i]=src->subimageSeqs()[i];
    msg.timestamp = src->timestamp();
    return msg;
  }
      

  

  LocalMap* msg2localMap(const LocalMapMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    // cerr<< "message read, id: " << id << endl;
    
    LocalMap* node;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      node = dynamic_cast<LocalMap*>(o);
    } else {
      node = new LocalMap;
      node->setId(id);
      context->add(node);
    } 
    if (! node) {
      // cerr<< "id: " << endl;
      throw std::runtime_error("aaaaaa");
    }
    node->setTransform(pose2eigen(msg.transform));
    /*
    for(size_t i = 0; i< msg.parents.size(); i++){
      boss::Identifiable* o=context->getById(msg.parents[i]);
      if (! o ) {
	throw std::runtime_error("invalid object instantiated");
      }
      MapNode* n=dynamic_cast<MapNode*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      node->parents().insert(n);
    }
    */

    std::list<MapNode*> nodes;
    for(size_t i = 0; i< msg.trajectory.size(); i++){
      // cerr<< "trj: " << msg.trajectory[i] << endl;
      boss::Identifiable* o=context->getById(msg.trajectory[i]);
      if (! o ) {
	return 0;
      }
      MapNode* n=dynamic_cast<MapNode*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      nodes.push_back(n);
    }
    node->nodes().clear();
    node->relations().clear();
    for (std::list<MapNode*>::iterator it = nodes.begin(); it!=nodes.end(); it++){
      MapNode* n = *it;
      node->nodes().addElement(n);
      n->parents().insert(node);
    }


    std::list<BinaryNodeRelation*> relations;
    for(size_t i = 0; i< msg.relations.size(); i++){
      // cerr<< "trj: " << msg.trajectory[i] << endl;
      boss::Identifiable* o=context->getById(msg.relations[i]);
      if (! o ) {
	return 0;
      }
      BinaryNodeRelation* n=dynamic_cast<BinaryNodeRelation*>(o);
      if (! n)
	throw std::runtime_error("wrong object type");
      relations.push_back(n);
    }
    for (std::list<BinaryNodeRelation*>::iterator it = relations.begin(); it!=relations.end(); it++){
      BinaryNodeRelation* rel = *it;
      node->relations().insert(std::tr1::shared_ptr<BinaryNodeRelation>(rel));
    }

    Cloud* c = new Cloud;
    msg2cloud(*c, msg.cloud);
    node->setCloud(c);
    node->cloudReference().setId(msg.cloud_id);
    // cerr<< "cloud id: " << msg.cloud_id << endl;
    node->cloudReference().setContext(context);
    context->add(&node->cloudReference());
    node->setTimestamp(msg.timestamp);
    return node;
  }

  LocalMapMsg localMap2msg(LocalMap* src, boss::IdContext* context) {
    LocalMapMsg msg;
    if (src->getId()==-1) {
      src->setId(context->generateId());
      // cerr<< "message construct, new id for local map: " << src->getId() << endl;
    } //else
      // cerr<< "message construct, local map already existing: " << src->getId() << endl;

    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.transform = eigen2pose(src->transform());
    msg.parents.resize(src->parents().size());
    int k=0;
    for (MapNodeSet::iterator it=src->parents().begin(); it!=src->parents().end(); it++){
      MapNode* n = *it;
      if (n->getId()==-1)
	n->setId(context->generateId());
      n->setContext(context);
      context->add(n);
      msg.parents[k] = n->getId();
      k++;
    }
    msg.trajectory.resize(src->nodes().size());
    k = 0;
    for(MapNodeList::iterator it = src->nodes().begin(); it!=src->nodes().end(); it++){
      msg.trajectory[k] = it->get()->getId();
      k++;
    }
    cloud2msg(msg.cloud, *src->cloud());
    if (src->cloudReference().getId()==-1) {
      msg.cloud_id = context->generateId();
      src->cloudReference().setId(msg.cloud_id);
      src->cloudReference().setContext(context);
      context->add(&src->cloudReference());
      // cerr<< "message construct, new id for cloud reference map: " << src->cloudReference().getId() << endl;
    } 
      // cerr<< "message construct, cloud reference map already existing: " << src->cloudReference().getId() << endl;
    msg.relations.resize(src->relations().size());
    k = 0;
    for (BinaryNodeRelationSet::iterator it=src->relations().begin(); it!=src->relations().end(); it++){
      BinaryNodeRelation* rel = it->get();
      if (rel->getId()==-1)
	rel->setId(context->generateId());
      rel->setContext(context);
      context->add(rel);
      msg.relations[k] = rel->getId();
      k++;
    }
   
    msg.timestamp = src->timestamp();
    return msg;
  }



  BinaryNodeRelation* msg2binaryNodeRelation(const BinaryNodeRelationMsg& msg, boss::IdContext* context) {
    int id = msg.id;
    // cerr<< "RELATION message read, id: " << id << endl;
    
    BinaryNodeRelation* rel;
    boss::Identifiable* o=context->getById(id);
    if (o){ 
      rel = dynamic_cast<BinaryNodeRelation*>(o);
    } else {
      rel = new BinaryNodeRelation;
      rel->setId(id);
      context->add(rel);
    } 
    if (! rel) {
      // cerr<< "id: " << endl;
      throw std::runtime_error("baaaaaaaad");
    }


    o=context->getById(msg.from_id);
    if (! o ) {
      // cerr<< "RELATION missing from: " << msg.from_id << endl;

      return 0;
    }
    MapNode* from=dynamic_cast<MapNode*>(o);
    if (! from)
      throw std::runtime_error("wrong object type");
    rel->setFrom(from);

    o=context->getById(msg.to_id);
    if (! o ) {
      // cerr<< "RELATION missing to: " << msg.to_id << endl;
      return 0;
    }
    MapNode* to=dynamic_cast<MapNode*>(o);
    if (! to)
      throw std::runtime_error("wrong object type");

    rel->setTo(to);

    if (msg.parent_id!= -1) {
      o=context->getById(msg.parent_id);
      if (! o ) {
	// cerr<< "RELATION missing to: " << msg.to_id << endl;
	return 0;
      }
      MapNode* parent=dynamic_cast<MapNode*>(o);
      if (! parent)
	throw std::runtime_error("wrong object type");
      rel->setParent(parent);
    } else
      rel->setParent(0);

    rel->setTransform(pose2eigen(msg.transform));

    int k = 0; 
    Matrix6f info;
    for(int r = 0; r<6; r++)
      for(int c = 0; c<6; c++)
	info(r,c)=msg.information_matrix[k++];
    rel->setInformationMatrix(info);
    // cerr<< "relation fuklly deserialized" << endl;
    return rel;
  }
  
  BinaryNodeRelationMsg binaryNodeRelation2msg(BinaryNodeRelation* src, boss::IdContext* context){
    BinaryNodeRelationMsg msg;
    if (src->getId()==-1) {
      src->setId(context->generateId());
    } 
    src->setContext(context);
    context->add(src);
    msg.id = src->getId();
    msg.from_id = src->from()->getId();
    msg.to_id = src->to()->getId();
    msg.parent_id = src->parent()? src->parent()->getId() : -1 ;
    msg.transform = eigen2pose(src->transform());
    int k=0;
    for (size_t r = 0; r< 6; r++)
      for (size_t c = 0; c< 6; c++){
	msg.information_matrix[k]=src->informationMatrix()(r,c);
	k++;
      }
    return msg;
  }

}
