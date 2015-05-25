#include "cloud_viewer.h"
#include <qevent.h>
#include <cstring>
#include "gl_helpers/opengl_primitives.h"
#include "gl_helpers/draw_attributes.h"

namespace fps_mapper{
  using namespace std;
  using namespace Eigen;
  using namespace GLHelpers;

  CloudViewer::CloudViewer(){
    _mode = MoveCamera;
    _is_orthographic = false;
  }

  void CloudViewer::addCloud(Cloud* c, const Eigen::Isometry3f& iso){
    _clouds.insert(make_pair(c,iso));
  }

  void CloudViewer::eraseCloud(Cloud* c) {
    _clouds.erase(c);
  }
  
  void CloudViewer::draw(){
    for (CloudIsometryMap::iterator it = _clouds.begin(); it!=_clouds.end(); it++) {
      const Cloud* cloud = it->first;
      int attrs = ATTRIBUTE_SHOW;
      glPointSize(3);
      if (_selected_objects.count(cloud)) {
	glColor3f(0.8, 0.8, 0.2);
	attrs |=ATTRIBUTE_SELECTED;
      } else {
	glColor3f(0.2, 0.4, 0.4);
      }
      glPushMatrix();
      GLHelpers::glMultMatrix(it->second);
      cloud->draw();
      glPopMatrix();
    }
  }

  void CloudViewer::drawWithNames() {
    _names_map.clear();
    int name = 0;
    for (CloudIsometryMap::iterator it = _clouds.begin(); it!=_clouds.end(); it++) {
      const Cloud* cloud = it->first;
      glPushMatrix();
      GLHelpers::glMultMatrix(it->second);
      cloud->draw(name);
      glPopMatrix();
      _names_map.insert(make_pair(name, cloud));
      name++;
    }
  }

  void CloudViewer::postSelection(const QPoint&){
    int id = selectedName();
    if (id < 0)
      return;
    const Cloud* cloud = _names_map[id];
    if (_selected_objects.count(cloud)){
      _selected_objects.erase(cloud);
    } else {
      _selected_objects.insert(cloud);
    }
  }


  void CloudViewer::keyPressEvent(QKeyEvent *e) {
    switch(e->key()){
    case Qt::Key_Space: 
      _mode = MoveCamera;
      cerr << "MoveCamera Mode is OFF" << endl;
      return;
    case Qt::Key_M:
      cerr << "MoveObject Mode is ON" << endl;
      _mode = MoveObject;
      return;
    case Qt::Key_O:
      _is_orthographic =  !_is_orthographic;
      if(_is_orthographic)  
	camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
      else
	camera()->setType(qglviewer::Camera::PERSPECTIVE);
      updateGL();
      return;
    default:;
    }
    if (_mode != MoveObject){
      SimpleViewer::keyPressEvent(e);
      return;
    }
    
    Eigen::Vector3f v(0,0,0);
    switch(e->key()){
    case Qt::Key_Up:
      v=Eigen::Vector3f(1,0,0);
      break;
    case Qt::Key_Down:
      v=Eigen::Vector3f(-1,0,0);
      break;
    case Qt::Key_Left:
      v=Eigen::Vector3f(0,1,0);
      break;
    case Qt::Key_Right:
      v=Eigen::Vector3f(0,-1,0);
      break;
    case Qt::Key_PageUp:
      v=Eigen::Vector3f(0,0,1);
      break;
    case Qt::Key_PageDown:
      v=Eigen::Vector3f(0,0,-1);
      break;
    default:
      SimpleViewer::keyPressEvent(e);
      return;
    }

    Eigen::Isometry3f transform;
    transform.setIdentity();
    float translational_step=0.1;
    float rotational_step = 0.1;
    if (e->modifiers()==Qt::ControlModifier){
      transform.linear() = Eigen::AngleAxisf(rotational_step, v).toRotationMatrix();
    } else {
      transform.translation() = v*translational_step;
    }

    for (std::set<const Cloud*>::iterator it=_selected_objects.begin(); 
	 it!=_selected_objects.end(); it++){
      Isometry3f& iso = _clouds[*it];
      iso = transform*iso;
    }
    updateGL();
  }

    
}

