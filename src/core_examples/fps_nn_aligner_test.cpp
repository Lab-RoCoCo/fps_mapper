#include "core/nn_aligner.h"
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

  voxelize(reference, 0.05);
  Cloud current;
  current.read(is2);
  cerr << "current has " << current.size() << " points" << endl;
  voxelize(current, 0.05);
  
  NNAligner aligner;
  aligner.setReferenceModel(&reference);
  aligner.setCurrentModel(&current);
  aligner.setIterations(100);
  aligner.align();
  
  current.transformInPlace(aligner.T());
  reference.add(current);
  ofstream os(argv[3]);
  reference.write(os);

  return 0;

}
