#include "boss/eigen_boss_plugin.h"
#include "map_node_list.h"
#include "GL/gl.h"
#include "gl_helpers/opengl_primitives.h"
#include <iostream>

namespace fps_mapper {
  using namespace std;

  void MapNodeList::resetBB() {
    _lower_translation = Eigen::Vector3f(1e3, 1e3, 1e3);
    _lower_orientation = Eigen::Vector3f(1e3, 1e3, 1e3);
    _upper_translation = Eigen::Vector3f(-1e3, -1e3, -1e3);
    _upper_orientation = Eigen::Vector3f(-1e3, -1e3, -1e3);
  }


  MapNodeList::MapNodeList() {
    resetBB();
  }

  MapNodeList::~MapNodeList() {
  }

  
  void MapNodeList::clear(){
    std::list<std::tr1::shared_ptr<MapNode> >::clear();
    resetBB();
  }

  void MapNodeList::draw(DrawAttributesType attributes, int name) {
    if (! (attributes&ATTRIBUTE_SHOW))
      return;
    // glBegin(GL_LINE_STRIP);
    // for(iterator it = begin(); it!=end(); it++) {
    //   glNormal3f(0,0,1);
    //   const Eigen::Vector3f v = (*it)->transform().translation();
    //   glVertex3f(v.x(), v.y(), v.z());
    // }
    // glEnd();
    if (attributes & ATTRIBUTE_SELECTED) {
      for(iterator it = begin(); it!=end(); it++) {
	(*it)->draw(attributes, name);
      }
    }
  }

  Eigen::Isometry3f  MapNodeList::meanPose() {
    if (empty())
      return Eigen::Isometry3f::Identity();

    Eigen::Vector3f orientation;
    Eigen::Vector3f translation;
    orientation.setZero();
    translation.setZero();
    int count = 0;
    for(iterator it = begin(); it!=end(); it++) {
      const Eigen::Isometry3f T = (*it)->transform();
      Eigen::AngleAxisf aa(T.linear());
      orientation+=aa.axis()*aa.angle();
      translation+=T.translation();
      count++;
    }
    orientation *= (1./count);
    translation *= (1./count);
    float angle = orientation.norm();
    Eigen::AngleAxisf aa;
    aa.angle()=angle;
    aa.axis()=orientation.normalized();
    Eigen::Isometry3f retval;
    retval.linear()=aa.toRotationMatrix();
    retval.translation()=translation;
    return retval;
  }


  Eigen::Isometry3f  MapNodeList::middlePose() {
    if (empty()) {
      return Eigen::Isometry3f::Identity();
    }
    Eigen::Vector3f translation;
    translation.setZero();
    int count = 0;
    for(iterator it = begin(); it!=end(); it++) {
      translation+=(*it)->transform().translation();
      count++;
    }
    translation *= (1./count);


    MapNode* best = 0;
    float best_distance = 1e9;
    for(iterator it = begin(); it!=end(); it++) {
      MapNode * current = it->get();
      float d = (current->transform().translation()-translation).squaredNorm();
      if ( ! best || d < best_distance){
	best = current;
	best_distance = d;
      }
    }
    return best->transform();
  }

  void MapNodeList::drawBox() {
    glBegin(GL_LINES);
    glVertex3f(_lower_translation.x(), _lower_translation.y(), _lower_translation.z());
    glVertex3f(_upper_translation.x(), _lower_translation.y(), _lower_translation.z());

    glVertex3f(_upper_translation.x(), _lower_translation.y(), _lower_translation.z());
    glVertex3f(_upper_translation.x(), _upper_translation.y(), _lower_translation.z());

    glVertex3f(_upper_translation.x(), _upper_translation.y(), _lower_translation.z());
    glVertex3f(_lower_translation.x(), _upper_translation.y(), _lower_translation.z());

    glVertex3f(_lower_translation.x(), _upper_translation.y(), _lower_translation.z());
    glVertex3f(_lower_translation.x(), _lower_translation.y(), _lower_translation.z());

    glVertex3f(_lower_translation.x(), _lower_translation.y(), _upper_translation.z());
    glVertex3f(_upper_translation.x(), _lower_translation.y(), _upper_translation.z());

    glVertex3f(_upper_translation.x(), _lower_translation.y(), _upper_translation.z());
    glVertex3f(_upper_translation.x(), _upper_translation.y(), _upper_translation.z());

    glVertex3f(_upper_translation.x(), _upper_translation.y(), _upper_translation.z());
    glVertex3f(_lower_translation.x(), _upper_translation.y(), _upper_translation.z());

    glVertex3f(_lower_translation.x(), _upper_translation.y(), _upper_translation.z());
    glVertex3f(_lower_translation.x(), _lower_translation.y(), _upper_translation.z());

    glVertex3f(_lower_translation.x(), _lower_translation.y(), _upper_translation.z());
    glVertex3f(_lower_translation.x(), _lower_translation.y(), _lower_translation.z());

    glVertex3f(_upper_translation.x(), _lower_translation.y(), _upper_translation.z());
    glVertex3f(_upper_translation.x(), _lower_translation.y(), _lower_translation.z());

    glVertex3f(_lower_translation.x(), _upper_translation.y(), _upper_translation.z());
    glVertex3f(_lower_translation.x(), _upper_translation.y(), _lower_translation.z());

    glVertex3f(_upper_translation.x(), _upper_translation.y(), _upper_translation.z());
    glVertex3f(_upper_translation.x(), _upper_translation.y(), _lower_translation.z());

    glEnd();

  }

  void MapNodeList::addElement(MapNode* tnode) {
    const Eigen::Vector3f& translation = tnode->transform().translation();
    // get the x axis of the orientation
    const Eigen::Vector3f& orientation = tnode->transform().linear().col(0);
    _lower_translation.x() = _lower_translation.x() < translation.x() ? _lower_translation.x() : translation.x();
    _lower_translation.y() = _lower_translation.y() < translation.y() ? _lower_translation.y() : translation.y();
    _lower_translation.z() = _lower_translation.z() < translation.z() ? _lower_translation.z() : translation.z();
    _upper_translation.x() = _upper_translation.x() > translation.x() ? _upper_translation.x() : translation.x();
    _upper_translation.y() = _upper_translation.y() > translation.y() ? _upper_translation.y() : translation.y();
    _upper_translation.z() = _upper_translation.z() > translation.z() ? _upper_translation.z() : translation.z();

    _lower_orientation.x() = _lower_orientation.x() < orientation.x() ? _lower_orientation.x() : orientation.x();
    _lower_orientation.y() = _lower_orientation.y() < orientation.y() ? _lower_orientation.y() : orientation.y();
    _lower_orientation.z() = _lower_orientation.z() < orientation.z() ? _lower_orientation.z() : orientation.z();
    _upper_orientation.x() = _upper_orientation.x() > orientation.x() ? _upper_orientation.x() : orientation.x();
    _upper_orientation.y() = _upper_orientation.y() > orientation.y() ? _upper_orientation.y() : orientation.y();
    _upper_orientation.z() = _upper_orientation.z() > orientation.z() ? _upper_orientation.z() : orientation.z();

    push_back(std::tr1::shared_ptr<MapNode>(tnode));
  }

}
