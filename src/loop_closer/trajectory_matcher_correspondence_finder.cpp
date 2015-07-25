#include "trajectory_matcher_correspondence_finder.h"
#include "trajectory_matcher_solver.h"

namespace fps_mapper {

  TrajectoryMatcherCorrespondenceFinder::TrajectoryMatcherCorrespondenceFinder(TrajectoryMatcherSolver* solver_) {
    _solver = solver_;
    _points_distance = 0.25f;
  }

  void TrajectoryMatcherCorrespondenceFinder::compute() {
  }

}
