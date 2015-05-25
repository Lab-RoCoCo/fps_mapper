#pragma once
#include "globals/defs.h"
#include "base_correspondence_finder.h"
#include "solver.h"
#include <flann/flann.hpp>

namespace fps_mapper {
  class NNCorrespondenceFinder: public BaseCorrespondenceFinder{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  public:
    //! ctor, p is the projector object used to compute the correspondences
    NNCorrespondenceFinder(Solver* s);

    //! dtor, destroys the index if created
    virtual ~NNCorrespondenceFinder();

    //! call this once before whenever you change the current cloud
    void init();

    //! overridden method from base class
    void compute();

    float normalScaling() const {return _normal_scaling;}
    void setNormalScaling(float ns)  {_normal_scaling =  ns;}

  protected:
    flann::Index< flann::L2<float> > *_index;
    int _knn;
    flann::Matrix<float> _reference_matrix;
    flann::Matrix<float> _current_matrix;
    flann::Matrix<int> _indices_matrix;
    flann::Matrix<float> _distances_matrix;
    float _normal_scaling;
    
    // working_area
    
    std::vector<float> _reference_points;
    std::vector<float> _current_points;
    std::vector<int> _current_indices;
    std::vector<float> _current_distances;
  };
}
