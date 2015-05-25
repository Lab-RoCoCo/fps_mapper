#pragma once
#include "solver.h"

#include <fstream>

namespace fps_mapper {

  class BaseAligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BaseAligner();
    virtual ~BaseAligner();


    inline const Cloud* currentModel() const { return _solver.currentModel();}

    inline const Cloud* referenceModel() const { return _solver.referenceModel();}
    
    virtual void setCurrentModel( const Cloud* m);
    virtual void setReferenceModel( const Cloud* m);

    virtual void setMaxDistance(float) = 0;
    virtual float maxDistance() const = 0;
    inline const Eigen::Isometry3f& T() const {return _solver.T();}

    inline const Matrix6f& informationMatrix() const {return _solver.informationMatrix();}

    virtual void align(const Eigen::Isometry3f& initial_guess=Eigen::Isometry3f::Identity(),
		       const Matrix6f& initial_guess_information=Matrix6f::Zero()) = 0;

    inline Solver& solver() { return _solver; }
  protected:

    Solver _solver;
  };

}
