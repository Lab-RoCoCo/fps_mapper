#include "globals/system_utils.h"
#include "core/depth_utils.h"
#include "core/nn_aligner.h"
#include "cloud_viewer.h"

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <iostream>
#include <Eigen/Core>

#include <limits>
#include <deque>
#include <queue>
#include <vector>
#include <fstream>
#include <qevent.h>
#include "qglviewer.h"
#include "qapplication.h"

#include <fstream>

using namespace std;
using namespace Eigen;
using namespace fps_mapper;
using namespace system_utils;


class AlignerViewer: public CloudViewer {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AlignerViewer(){
  }


  void keyPressEvent(QKeyEvent *e) {
    if (e->key()==Qt::Key_X){
      align();
      return;
    }
    if (e->key()==Qt::Key_P){
      cerr << "transforms: " << endl;
      for (CloudIsometryMap::iterator it = _clouds.begin(); it!=_clouds.end(); it++) {
	cerr << it->first << " -> " << t2v(it->second).transpose() << endl;
      }
      return;
    }
    CloudViewer::keyPressEvent(e);
  }

  void align(){
    if (_selected_objects.size()!=2) {
      cerr << "the objects should be exactly two";
      return;
    }
    
    std::set<const Cloud*>:: iterator it = _selected_objects.begin();
    Cloud reference = *(*it);
    Eigen::Isometry3f& reference_transform = _clouds[*it];
    it++;
    Cloud current = *(*it);
    Cloud curr_cloud = *(*it);
    Eigen::Isometry3f& current_transform = _clouds[*it];

    voxelize(reference, 0.05);
    voxelize(current, 0.05);
    
    // determine the transform between reference and current
    Eigen::Isometry3f dt = reference_transform.inverse() * current_transform;
    
    aligner.setReferenceModel(&reference);
    aligner.setCurrentModel(&current);
    aligner.setIterations(10);
    aligner.align(dt);
    aligner.solver().setDamping(1e3);
    aligner.solver().setMaxError(0.01);
    current_transform = reference_transform*aligner.T();
    updateGL();

    curr_cloud.transformInPlace(current_transform);
    ofstream os("refined.cloud");
    curr_cloud.write(os);
    printErrorStats();
  }

  void printErrorStats() {
    const std::vector<float>& errors = aligner.solver().errors();
    int inliers = 0;
    double inliers_error_sum=0;
    double outliers_error_sum=0;
    int outliers = 0;
    for (size_t i = 0; i<errors.size(); i++){
      if (errors [i] <0 ){
	outliers++;
	outliers_error_sum -= errors[i];
	continue;
      } else {
	inliers++;
	inliers_error_sum += errors[i];
      }
    }
    cerr << "solver.max_error: " << aligner.solver().maxError() << endl;
    cerr << "inliers : " << inliers << endl;
    cerr << "outliers: " << outliers << endl;
    cerr << "error/inliers: " << inliers_error_sum/inliers << endl;
    cerr << "error/outliers: " << outliers_error_sum/outliers << endl;
  }

  NNAligner aligner;
};


const char* banner[]= {
  "fps_aligner_gui_app",
  "allows to align a set of clouds",
  "usage:",
  " fps_aligner_gui_app <clouds>",
  "",
  " once the giu has started",
  " shift-click selects a cloud",
  " M toggles the move cloud mode with cursor and page up keys (ctrl to rotate)",
  " X aligns two clouds",
  " P prints the relative transforms of the clouds",
  0
};


int main(int argc, char **argv) {
  std::list<Cloud*> clouds;
  if (argc<2 || ! strcmp(argv[1],"-h")){
    printBanner(banner);
    return 0;
  }

  int c = 1;
  while (c<argc){
    Cloud* cloud=new Cloud;
    ifstream is(argv[c]);
    cloud->read(is);
    clouds.push_back(cloud);
    cerr << "loaded cloud [" << argv[c] << "] with " << cloud->size() << "] points" << endl;
    c++;
  }
  QApplication app(argc, argv);
  AlignerViewer viewer;
  for (std::list<Cloud*>::iterator it=clouds.begin(); it!=clouds.end(); it++)
    viewer.addCloud(*it);
  viewer.show();
  app.exec();
}
