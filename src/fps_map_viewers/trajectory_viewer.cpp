#include "trajectory_viewer.h"
#include "qevent.h"
#include <cstring>
#include "gl_helpers/opengl_primitives.h"

namespace fps_mapper{
  using namespace std;
  using namespace Eigen;
  using namespace GLHelpers;
  
  void TrajectoryViewer::draw() {
    int k = 0;
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
      MapNode* n = it->get();
      int attrs = ATTRIBUTE_SHOW;

      if (_selected_objects.count(n)) {
	glColor3f(0.8, 0.5, 0.5);
	attrs |=ATTRIBUTE_SELECTED;
      } else {
	glColor3f(0.5, 0.8, 0.5);
      }
      n->draw(attrs);
      k++;
    }
    nodes.draw();
    for (BinaryNodeRelationSet::iterator it = relations.begin(); it!=relations.end(); it++) {
      if (! (*it)->parent())
	(*it)->draw();
    }

  }

  void TrajectoryViewer::drawWithNames() {
    _names_map.clear();
    int name = 0;
    for (MapNodeList::iterator it = nodes.begin(); it!=nodes.end(); it++) {
      MapNode* n = it->get();
      n->draw(ATTRIBUTE_SHOW, name);
      _names_map.insert(make_pair(name, n));
      name++;
    }
  }

  void TrajectoryViewer::postSelection(const QPoint&){
    int id = selectedName();
    cerr << "The id of the selected object is" << id << endl;
    if (id < 0)
      return;
    MapNode* node = _names_map[id];
    if (_selected_objects.count(node)) 
      _selected_objects.erase(node);
    else
      _selected_objects.insert(node);
  }

}

