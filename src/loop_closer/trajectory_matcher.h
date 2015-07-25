#pragma once

#include "trajectory_matcher_correspondence_finder.h"
#include "trajectory_matcher_solver.h"

namespace fps_mapper {
  
  class TrajectoryMatcher {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    TrajectoryMatcher();
    virtual ~TrajectoryMatcher() {}

    inline int iterations() const {return _iterations; }
    inline const LocalMap* currentLocalMap() const { return _solver.currentLocalMap(); }
    inline const LocalMap* referenceLocalMap() const { return _solver.referenceLocalMap(); }
   
    inline void setIterations(int i) { _iterations = i; }
    virtual void setCurrentLocalMap(const LocalMap* lmap) { _solver.setCurrentLocalMap(lmap); }
    virtual void setReferenceLocalMap(const LocalMap* lmap) { _solver.setReferenceLocalMap(lmap); }

    inline TrajectoryMatcherCorrespondenceFinder& finder() { return _finder; }
    inline TrajectoryMatcherSolver& solver() { return _solver; }    
    inline const Eigen::Isometry3f& T() const { return _solver.T(); }

    virtual void align(const Eigen::Isometry3f& initial_guess = Eigen::Isometry3f::Identity());

  protected:
    int _iterations;
    TrajectoryMatcherCorrespondenceFinder _finder;
    TrajectoryMatcherSolver _solver;

  };
}
