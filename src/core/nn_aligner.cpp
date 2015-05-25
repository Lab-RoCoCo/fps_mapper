#include "nn_aligner.h"
#include <stdexcept>

namespace fps_mapper {

using namespace std;
using namespace Eigen;


  NNAligner::NNAligner():
    _finder(&_solver) {
    _solver.setMaxError(.01);
    _solver.setDamping(100);
    _solver.setGICP(true);
    _finder.setPointsDistance(0.3);
    _iterations = 10;
    _max_distance = 3;
  }

  NNAligner::~NNAligner() {}



  float NNAligner::maxDistance() const {return _max_distance;}

  void NNAligner::setMaxDistance(float md)  {_max_distance = md;}

  void NNAligner::align(const Eigen::Isometry3f& initial_guess,
 			const Matrix6f& initial_guess_information){
    if (! _solver.referenceModel()) {
      throw std::runtime_error("NNAligner: align(), reference model not set");
    }
    if (!_solver.currentModel()){
      throw std::runtime_error("NNAligner: align(), current model not set");
    }
    if (initial_guess_information==Matrix6f::Zero()) {
      _solver.setT(initial_guess);
    } else 
      _solver.setT(initial_guess,initial_guess_information);

    _finder.init();
    for(int i = 0; i<_iterations; i++){
	_finder.compute();
	const BaseCorrespondenceFinder::CorrespondenceVector& corr=_finder.correspondences();
	bool computeStats = (i == _iterations-1);
	_solver.oneRound(corr, computeStats);
    }
  }
}
