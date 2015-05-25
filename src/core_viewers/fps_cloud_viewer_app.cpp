#include "globals/system_utils.h"
#include "core/depth_utils.h"
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
#include "qglviewer.h"
#include "qapplication.h"



using namespace std;
using namespace fps_mapper;
using namespace Eigen;
using namespace system_utils;

const char* banner[]= {
  "fps_cloud_viewer_app",
  "shows a bunch of clouds, shift click to select",
  "usage:",
  " fps_cloud_viewer_app <clouds>",
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
  CloudViewer viewer;
  for (std::list<Cloud*>::iterator it=clouds.begin(); it!=clouds.end(); it++)
    viewer.addCloud(*it);
  viewer.show();
  app.exec();
}
