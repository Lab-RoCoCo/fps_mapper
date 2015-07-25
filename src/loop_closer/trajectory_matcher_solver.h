#pragma once

#include "fps_map/local_map.h"

#include "trajectory_matcher_correspondence_finder.h"

namespace fps_mapper {

  class TrajectoryMatcherSolver {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef Eigen::Matrix<float, 3, 6> Matrix3_6f;

    TrajectoryMatcherSolver();
    virtual ~TrajectoryMatcherSolver() {}

    void setReferenceLocalMap(const LocalMap* lmap) { _reference = lmap; }
    void setCurrentLocalMap(const LocalMap* lmap) { _current = lmap; }
    inline void setT(const Eigen::Isometry3f& T_) { _T = T_; }
    inline void setDamping(const float damping_) { _damping = damping_; }

    inline const LocalMap* referenceLocalMap() const { return _reference; }
    inline const LocalMap* currentLocalMap() const { return _current; }
    inline const Matrix6f& H() const { return _H; }
    inline const Vector6f& b() const { return _b; } 
    inline const Eigen::Isometry3f& T() const { return _T; }    
    inline float error() const { return _error; }
    inline float damping() const { return _damping; }

    void errorAndJacobianPoint(Eigen::Vector3f& pointError, Matrix3_6f& J, 
			       const Eigen::Vector3f& referencePoint, 
			       const Eigen::Vector3f& currentPoint) const;
    virtual void linearize(const TrajectoryMatcherCorrespondenceFinder::CorrespondenceVector& correspondences);
    virtual void oneRound(const TrajectoryMatcherCorrespondenceFinder::CorrespondenceVector& correspondences);

  protected:
    float _error, _damping;
    Vector6f _b;
    Matrix6f _H;
    Eigen::Isometry3f _T;
    const LocalMap* _reference;
    const LocalMap* _current;

 };

}
