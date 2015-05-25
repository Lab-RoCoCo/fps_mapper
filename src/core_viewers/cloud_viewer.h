#pragma once

#include "core/cloud.h"
#include "gl_helpers/simple_viewer.h"

namespace fps_mapper {

  class CloudViewer: public GLHelpers::SimpleViewer{
  public:
    typedef std::map<const Cloud*, Eigen::Isometry3f, std::less<const Cloud*>,
		     Eigen::aligned_allocator<std::pair<const Cloud*, Eigen::Isometry3f> > > CloudIsometryMap;
    CloudViewer();
    enum Mode {MoveCamera=0x0, MoveObject=0x1};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    virtual void draw();
    virtual void drawWithNames();
    virtual void keyPressEvent(QKeyEvent *e);

    void addCloud(Cloud* c, const Eigen::Isometry3f& iso = Eigen::Isometry3f::Identity());
    void eraseCloud(Cloud* c);
  protected:
    CloudIsometryMap _clouds;
    Mode _mode;
    std::map<int, const Cloud*> _names_map;
    virtual void postSelection(const QPoint& point);
    std::set<const Cloud*> _selected_objects;
    bool _is_orthographic;
  };


}
