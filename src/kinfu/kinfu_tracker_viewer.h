#pragma once

#include "qglviewer.h"
#include "qapplication.h"
#include "fps_kinfu_tracker.h"
#include "gl_helpers/simple_viewer.h"

namespace fps_mapper {

  class KinfuTrackerViewer: public GLHelpers::SimpleViewer {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    KinfuTrackerViewer(FPSKinfuTracker* _tracker);

    virtual void draw();

  protected:
    FPSKinfuTracker* _tracker;
    bool _model_tainted;
  };


}
