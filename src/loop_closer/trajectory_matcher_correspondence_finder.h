#pragma once

#include <utility>

#include "fps_map/map_node.h"

namespace fps_mapper {

  class TrajectoryMatcherSolver;

  class TrajectoryMatcherCorrespondenceFinder {
  public:
    typedef std::pair<std::tr1::shared_ptr<MapNode>, std::tr1::shared_ptr<MapNode> > Correspondence;
    typedef std::vector<Correspondence> CorrespondenceVector;
    
    TrajectoryMatcherCorrespondenceFinder(TrajectoryMatcherSolver* solver_);
    virtual ~TrajectoryMatcherCorrespondenceFinder() {}

    inline float pointsDistance() const { return _points_distance; }
    inline const CorrespondenceVector& correspondences() const { return _correspondences; }
    inline TrajectoryMatcherSolver& solver() { return *_solver; }

    inline void setPointsDistance(float points_distance_) { _points_distance = points_distance_; }

    virtual void compute();

  protected:
    float _points_distance;
    CorrespondenceVector _correspondences;
    TrajectoryMatcherSolver* _solver;

  };

}
