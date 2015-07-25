#pragma once

#include "core/base_aligner.h"
#include "fps_map/local_map.h"

namespace fps_mapper {

   class BaseLoopCloser {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      BaseLoopCloser(BaseAligner* aligner_) { this->setAligner(aligner_); }
      virtual ~BaseLoopCloser() {}
  
      inline BaseAligner* aligner() { return _aligner; }
      inline void setAligner(BaseAligner* aligner_) { _aligner = aligner_; }

      virtual void compute(MapNodeList& candidateClosures, MapNodeList& closures,
			   BinaryNodeRelationSet& relations, 
			   LocalMap* currentLocalMap, MapNodeList* localMaps) = 0;
      virtual BinaryNodeRelation* matchLocalMaps(LocalMap& reference, LocalMap& current, 
						 Eigen::Isometry3f initialGuess = Eigen::Isometry3f::Identity(), 
						 float voxelLeaf = 0.05f);
      
   protected:
      BaseAligner* _aligner;
   };

}
