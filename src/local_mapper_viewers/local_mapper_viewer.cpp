#include "local_mapper_viewer.h"
#include "qevent.h"
#include <cstring>
#include "gl_helpers/opengl_primitives.h"

namespace fps_mapper{
  using namespace std;
  using namespace Eigen;
  using namespace GLHelpers;
 
  LocalMapperViewer::LocalMapperViewer(LocalMapTrigger* t) {
    _trigger = t;
    _tracker = t->tracker();
    _trigger->setLocalMaps(&nodes);
    _trigger->setLocalMapsRelations(&relations);
    _modelTainted = true;
    _local_map_trajectory = _trigger->nodes();
  }
  
  void LocalMapperViewer::draw(){
    if (!_tracker->currentModel() || !_tracker->referenceModel() )
      return;

    glPushAttrib(GL_COLOR);
    glPushMatrix();
    glMultMatrix( _tracker->globalT() );

    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    GLHelpers::drawReferenceSystem();
    glPopMatrix();

   /// draw the current after applying the epsilon T from aligner
    glPushMatrix();
    glMultMatrix(_tracker->aligner().T().inverse() );
    if (_tracker->currentModel()){
      glColor3f(0.3, 0.3, 0.8);
      _tracker->currentModel()->draw();
    }
    glPopMatrix();

    glPushMatrix();
    glMultMatrix(_tracker->lastCamera()->offset());
    float pyrH = 0.1;
    float pyrW = 0.05;
    drawPyramidWireframe(/*pyrH = */ 0.1, /*pyrW = */ 0.05);
    glPopMatrix();

    /// draw the reference
    if (_tracker->referenceModel()){
      glColor3f(0.5, 0.5, 0.5);
      _tracker->referenceModel()->draw();
    }
 
    glPopMatrix();
    glPopAttrib();
    
    TrajectoryViewer::draw();

    glPushAttrib(GL_COLOR);
    if (_local_map_trajectory) {
      for (MapNodeList::iterator it = _local_map_trajectory->begin(); it!=_local_map_trajectory->end();
	   it++) 
	(*it)->draw();
    }
    glPopAttrib();
  }

}

