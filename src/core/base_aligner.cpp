#include "base_aligner.h"

namespace fps_mapper {

  BaseAligner::BaseAligner(){
    _solver.setMaxError(.01);
    _solver.setDamping(100);
    _solver.setGICP(true);
  }

  BaseAligner::~BaseAligner(){}
  

  void BaseAligner::setCurrentModel( const Cloud* m) {
    _solver.setCurrentModel(m);
  }
  
  void BaseAligner::setReferenceModel( const Cloud* m) {
    _solver.setReferenceModel(m);
  }

}
