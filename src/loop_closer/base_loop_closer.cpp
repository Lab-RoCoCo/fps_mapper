#include "base_loop_closer.h"

namespace fps_mapper {

  BinaryNodeRelation* BaseLoopCloser::matchLocalMaps(LocalMap& reference, LocalMap& current, 
						     Eigen::Isometry3f initialGuess, 
						     float voxelLeaf) {
    Cloud tempRef(*reference.cloud());
    Cloud tempCurr(*current.cloud());
    voxelize(tempRef, voxelLeaf);
    voxelize(tempCurr, voxelLeaf);
    _aligner->setReferenceModel(&tempRef);
    _aligner->setCurrentModel(&tempCurr);
    _aligner->align(initialGuess);
    BinaryNodeRelation* rel = new BinaryNodeRelation(&reference, &current, _aligner->T(), Matrix6f::Identity());
    return rel;
  }

}
