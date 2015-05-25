#include <cstring>

#include "gl_helpers/opengl_primitives.h"

#include "kinfu_tracker_viewer.h"
#include "qevent.h"

namespace fps_mapper{

  using namespace std;
  using namespace Eigen;
  using namespace GLHelpers;
 
  KinfuTrackerViewer::KinfuTrackerViewer(FPSKinfuTracker* t) {
    _tracker = t;
    _model_tainted = true;
  }
  
  void KinfuTrackerViewer::draw() {
    if(!_tracker->currentModel() || !_tracker->referenceModel() ) { return; }

    glPushMatrix();
    glMultMatrix(_tracker->globalTransform());

    glPushMatrix();
    glScalef(0.2, 0.2, 0.2);
    GLHelpers::drawReferenceSystem();
    glPopMatrix();

    // draw the reference
    glColor3f(0.5, 0.5, 0.5);
    _tracker->referenceModel()->draw();
    
    // draw the current after applying the epsilon T from aligner
    glPushMatrix();
    glMultMatrix(_tracker->deltaTransform());
    glColor3f(0.2, 0.2, 1.0);
    _tracker->currentModel()->draw();
    glPopMatrix();

    glPopMatrix();
  }

}

