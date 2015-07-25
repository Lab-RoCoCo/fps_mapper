#include "trajectory_matcher_solver.h"

namespace fps_mapper {

  TrajectoryMatcherSolver::TrajectoryMatcherSolver() {
    _H.setZero();
    _b.setZero();
    _T.setIdentity();
    _error = 0;
    _reference = 0;
    _current = 0;
    _damping = 100;
  }

  void TrajectoryMatcherSolver::errorAndJacobianPoint(Eigen::Vector3f& pointError, Matrix3_6f& J, 
						      const Eigen::Vector3f& referencePoint, 
						      const Eigen::Vector3f& currentPoint) const {
  }

  void TrajectoryMatcherSolver::linearize(const TrajectoryMatcherCorrespondenceFinder::CorrespondenceVector& correspondences) {
  }

  void TrajectoryMatcherSolver::oneRound(const TrajectoryMatcherCorrespondenceFinder::CorrespondenceVector& correspondences) {
  }

}
