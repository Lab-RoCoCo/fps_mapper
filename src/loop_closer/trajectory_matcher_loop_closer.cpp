#include "trajectory_matcher_loop_closer.h"

#include <opencv2/highgui/highgui.hpp>

#include "core/nn_aligner.h"
#include "fps_map/multi_image_map_node.h"
#include "core/depth_utils.h"

#include "core/pinhole_camera_info.h"

using namespace Eigen;

namespace fps_mapper {

  struct neighborCandidate {
    neighborCandidate(std::tr1::shared_ptr<MapNode> mapNode_, Eigen::Isometry3f initialGuess_) {
      mapNode = mapNode_;
      initialGuess = initialGuess_;
    }
    
    std::tr1::shared_ptr<MapNode> mapNode;
    Eigen::Isometry3f initialGuess;
  };

  TrajectoryMatcherLoopCloser::TrajectoryMatcherLoopCloser(BaseAligner* aligner_):
    BaseLoopCloser(aligner_) { 
    _max_energy = 0.025f;    
    _candidates_ratio = 0.1f;
    _max_distance = 5.0f;
    _max_bad_points_ratio = 0.1f;
    _projector = 0;
    _aligner->solver().setMaxError(0.01);
  }

  void TrajectoryMatcherLoopCloser::compute(MapNodeList& candidateClosures,
					    MapNodeList& closures,
					    BinaryNodeRelationSet& relations, 
					    LocalMap* currentLocalMap, 
					    MapNodeList* localMaps) {
  }

  float TrajectoryMatcherLoopCloser::matchTrajectories(LocalMap* currentLocalMap, LocalMap* candidateLocalMap,
						       Eigen::Isometry3f& T) {
								   return 0.0;	// dummy value, original code has been removed because of missing copyright permissio
  }

  bool TrajectoryMatcherLoopCloser::geometryCheck(std::tr1::shared_ptr<BinaryNodeRelation>& relation,
						  LocalMap* currentLocalMap, LocalMap* candidateLocalMap, 
						  Eigen::Isometry3f& T) {
							  return false; // dummy value, original code has been removed because of missing copyright permission
  }

  void TrajectoryMatcherLoopCloser::_setProjectorSize(int& rows, int& cols) {
  }

  void TrajectoryMatcherLoopCloser::_computeConsistency(const FloatImage& currentDepth, 
							const IntImage& currentIndices,
							const FloatImage& candidateDepth, 
							const IntImage& candidateIndices) {
  }

}

