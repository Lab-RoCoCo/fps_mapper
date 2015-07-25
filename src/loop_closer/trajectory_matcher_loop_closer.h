#pragma once

#include "core/multi_projector.h"

#include "base_loop_closer.h"
//#include "trajectory_matcher.h"

namespace fps_mapper {

  class TrajectoryMatcherLoopCloser: public BaseLoopCloser {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW    

    TrajectoryMatcherLoopCloser(BaseAligner* aligner_); 
    virtual ~TrajectoryMatcherLoopCloser() { delete _projector; }

    inline float maxEnergy() const { return _max_energy; }
    inline float candidatesRatio() const { return _candidates_ratio; }
    inline float maxDistance() const { return _max_distance; }
    inline float maxBadPointsRatio() const { return _max_bad_points_ratio; }
    inline float maxBadPointsDistance() const { return _max_bad_points_distance; }
    //inline TrajectoryMatcher& trajectoryMatcher() { return _trajectory_matcher; }

    inline void setMaxEnergy(const float max_energy_) { _max_energy = max_energy_; }
    inline void setMaxDistance(const float max_distance_) { _max_distance = max_distance_; }
    inline void setMaxBadPointsDistance(const float max_bad_points_distance_) { _max_bad_points_distance = max_bad_points_distance_; }
    inline void setMaxBadPointsRatio(const float max_bad_points_ratio_) { _max_bad_points_ratio = max_bad_points_ratio_; }
    inline void setCandidatesRatio(const float candidates_ratio_) { _candidates_ratio = candidates_ratio_; }
    //inline void setTrajectoryMatcher(TrajectoryMatcher trajectory_matcher_) { _trajectory_matcher = trajectory_matcher_; }
 
    virtual void compute(MapNodeList& candidateClosures, MapNodeList& closures,
			 BinaryNodeRelationSet& relations, 
			 LocalMap* currentLocalMap, MapNodeList* localMaps);    
    float matchTrajectories(LocalMap* currentLocalMap, LocalMap* candidateLocalMap, 
			    Eigen::Isometry3f& T);
    bool geometryCheck(std::tr1::shared_ptr<BinaryNodeRelation>& relation,
		       LocalMap* currentLocalMap, LocalMap* candidateLocalMap, Eigen::Isometry3f& T);
	
  protected:    
    void _setProjectorSize(int& rows, int& cols);
    void _computeConsistency(const FloatImage& currentDepth, const IntImage& currentIndices,
			     const FloatImage& candidateDepth, const IntImage& candidateIndices);

    float _max_energy, _candidates_ratio, _max_distance, _max_bad_points_distance, _max_bad_points_ratio;
    float _in_distance, _out_distance;
    int _in_num, _out_num;
    MultiProjector* _projector;
    //TrajectoryMatcher _trajectory_matcher;
  };

}
