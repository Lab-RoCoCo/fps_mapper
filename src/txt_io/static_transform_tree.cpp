#include <string>
#include <map>
#include <memory>
#include "static_transform_tree.h"
#include <stdexcept>
#include "globals/defs.h"
#include "message_reader.h"

namespace txt_io{
  using namespace std;
  using namespace Eigen;
  StaticTransformTree::StaticTransformTree() {
    _root_frame_id = "";
  }


  void StaticTransformTree::applyTransform(BaseSensorMessage& msg) {
    Eigen::Isometry3f offset = msg.offset();
    getOffset(offset, msg.frameId());
    msg.setOffset(offset);
  }
 
  void StaticTransformTree::load(const std::string& filename) {
    _tree.clear();
    _root_frame_id = "";
    cerr << "Reading transforms from file" << filename << endl; 
    MessageReader tf_reader;
    tf_reader.open(filename);
    while (tf_reader.good()){
      BaseMessage* msg = tf_reader.readMessage();
      if (! msg)
	continue;
      StaticTransformMessage* tf_msg = dynamic_cast<StaticTransformMessage*>(msg);
      if (tf_msg)
	addMessage(tf_msg);
    }
    isWellFormed();
  }

  void StaticTransformTree::addMessage(StaticTransformMessage* msg) {
    StaticTransformMessage* other = link(msg->toFrameId());
    if (other) {
      cerr << "error, another transform leading to frame_id " << msg->toFrameId() << " exists" << endl;
      throw std::runtime_error("fatal");
    }
    StaticTransformMessage* parent = link(msg->fromFrameId());
    if (! parent) {
      _root_frame_id = msg->fromFrameId();
    } else {
      msg->setParent(parent);
    }
    _tree.insert(std::make_pair(msg->toFrameId(), std::tr1::shared_ptr<StaticTransformMessage>(msg)));
  }

  bool StaticTransformTree::isWellFormed() {
    bool fail = false;
    for (StringTransformMap::iterator it = _tree.begin(); it!=_tree.end(); it++) {
      Eigen::Isometry3f o;
      if (! getOffset(o, it->second->toFrameId())) {
	cerr << "missing transform from " << it->second->toFrameId() << " to " << " _root_frame_id" << endl;
	fail = true;
      } else {
	cerr << "transform from [" << _root_frame_id << "] to [" 
	     << it->second->toFrameId() << "] " << t2v(o).transpose() << endl;
      }
    }
    return fail;
  }

  bool StaticTransformTree::getOffset(Eigen::Isometry3f &offset, const std::string& frame_id) {
    Eigen::Isometry3f T;
    T.setIdentity();
    StaticTransformMessage* msg = link(frame_id);
    if (! msg)
      return false;
    while (msg){
      T = msg->transform()*T;
      if (msg->fromFrameId()==_root_frame_id){
	offset = T;
	return true;
      }
      msg=link(msg->fromFrameId());
    }
    return false;
  }

  StaticTransformMessage* StaticTransformTree::link(const std::string& s) {
    StringTransformMap::iterator it = _tree.find(s);
    if (it==_tree.end())
      return 0;
    return it->second.get();
  }

};
