#include "core/projective_aligner.h"
#include "core/depth_utils.h"

#include <fstream>

using namespace std;
using namespace fps_mapper;

int main(int argc, char** argv) {
  if (argc<4) {
    cerr << "usage: " << argv[0] << "<model1.dat> <model2.dat> <output.dat>" << endl;
    return 0;
  }

  ifstream is1(argv[1]);
  if(! is1) {
    cerr << "unable to load file " << argv[1] << endl;
    return 0;
  }

  ifstream is2(argv[2]);
  if(! is2) {
    cerr << "unable to load file " << argv[2] << endl;
    return 0;
  }

  cerr << "loading models" << endl;
  Cloud reference;
  reference.read(is1);
  cerr << "reference has " << reference.size() << " points" << endl;

  Cloud current;
  current.read(is2);
  cerr << "current has " << current.size() << " points" << endl;
  
  ProjectiveAligner aligner;
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.setDefaultConfig("Xtion640x480");
  aligner.align();
  
  float factor = 255.0f/6;

  FloatImage img = aligner.finder().zBuffer()-aligner.finder().referenceZBuffer();
  cv::imwrite("differences.png", img*1000);
  cv::imwrite("current.png", aligner.finder().zBuffer()*factor);
  cv::imwrite("reference.png", aligner.finder().referenceZBuffer()*factor);
  float in_distance, out_distance;
  int in_num, out_num;
  FloatImage items;
  compareDepths(in_distance, in_num, out_distance, out_num,
		aligner.finder().referenceZBuffer(), aligner.finder().referenceIndices(),
		aligner.finder().zBuffer(), aligner.finder().indices(),
		0.02, false, &items);

  cv::imwrite("items.png", items*factor);
  
  current.transformInPlace(aligner.T());
  reference.add(current);
  ofstream os(argv[3]);
  reference.write(os);

  return 0;

}
