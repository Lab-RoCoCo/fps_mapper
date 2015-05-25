#pragma once
#include "projective_correspondence_finder.h"
#include "solver.h"
#include "base_aligner.h"

#include <fstream>

namespace fps_mapper {

  class ProjectiveAligner : public BaseAligner {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct LevelParameters{
      LevelParameters(int s=1, int it=10) {
	scale = s;
	iterations = it;
      }
      int scale;
      int iterations;
    };
    //! ctor, if the projector is left null it creates its own
    ProjectiveAligner(BaseProjector* projector=0);
    ~ProjectiveAligner();

    static std::vector<std::string> defaultConfigs();

    void setDefaultConfig(const std::string config);

    inline std::vector<LevelParameters>& levelParameters() {return _level_parameters;}

    virtual float maxDistance() const;

    virtual void setMaxDistance(float md);

    virtual void align(const Eigen::Isometry3f& initial_guess=Eigen::Isometry3f::Identity(),
		       const Matrix6f& initial_guess_information=Matrix6f::Zero());

    inline ProjectiveCorrespondenceFinder& finder() { return _finder; }
    inline BaseProjector& projector() { return *_projector; }

  protected:
    std::vector<LevelParameters> _level_parameters;

    BaseProjector* _projector;
    ProjectiveCorrespondenceFinder _finder;

    FloatImage _zbuffer;
    IndexImage _indices;
    float _reference_canvas_scale;

    FloatImage _reference_buffer_hres;
    IndexImage _reference_indices_hres;
    FloatImage _current_buffer_hres;
    IndexImage _current_indices_hres;


    static void _initDefaultConfigs();
    static std::map<std::string, std::vector<LevelParameters> > _default_configs;
  };

}
