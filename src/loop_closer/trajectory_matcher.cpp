#include "trajectory_matcher.h"

namespace fps_mapper {
  
  TrajectoryMatcher::TrajectoryMatcher(): _finder(&_solver) {
    _solver.setDamping(100);
    _finder.setPointsDistance(1.0f);
    _iterations = 100;
  }

  void TrajectoryMatcher::align(const Eigen::Isometry3f& initial_guess) {
    if(!_solver.referenceLocalMap()) { 
      throw std::runtime_error("[WARNING]: TrajectoryMatcher's reference model not set"); }
    if(!_solver.currentLocalMap()) {
      throw std::runtime_error("[WARNING]: TrajectoryMatcher's current model not set");
    }

    _solver.setT(initial_guess);
    for(int i = 0; i < _iterations; i++) {
      _finder.compute();
      const TrajectoryMatcherCorrespondenceFinder::CorrespondenceVector& corr = _finder.correspondences();
      _solver.oneRound(corr);
    }
  }

}
