#include "projective_aligner.h"
#include "pinhole_projector.h"
#include "depth_utils.h"
#include <stdexcept>

#define _USE_IMAGE_SHRINK

namespace fps_mapper {

using namespace std;
using namespace Eigen;

  std::map<std::string, std::vector<ProjectiveAligner::LevelParameters> > ProjectiveAligner::_default_configs;

  float ProjectiveAligner::maxDistance() const {return _projector->maxDistance();}

  void ProjectiveAligner::setMaxDistance(float md)  {_projector->setMaxDistance(md);}

  void ProjectiveAligner::_initDefaultConfigs(){
    if(!_default_configs.empty())
      return;
    std::vector<LevelParameters> params;

    params.push_back(LevelParameters(1,0));
    _default_configs.insert(make_pair("do_nothing", params));

    params.push_back(LevelParameters(20,3));
    params.push_back(LevelParameters(10,3));
    _default_configs.insert(make_pair("conservative", params));

    params.push_back(LevelParameters(16,3));
    params.push_back(LevelParameters(8,3));
    params.push_back(LevelParameters(4,3));
    _default_configs.insert(make_pair("Kinect640x480", params));
    _default_configs.insert(make_pair("Xtion640x480", params));
    
    params.clear();
    params.push_back(LevelParameters(8,3));
    params.push_back(LevelParameters(4,3));
    params.push_back(LevelParameters(2,3));
    _default_configs.insert(make_pair("Xtion320x240", params));

    params.clear();
    params.push_back(LevelParameters(8,10));
    params.push_back(LevelParameters(4,10));
    params.push_back(LevelParameters(2,10));
    _default_configs.insert(make_pair("Xtion320x240-many", params));

    params.clear();
    params.push_back(LevelParameters(8,3));
    params.push_back(LevelParameters(4,3));
    params.push_back(LevelParameters(2,3));
    params.push_back(LevelParameters(1,3));
    _default_configs.insert(make_pair("4Levels", params));

    params.clear();
    params.push_back(LevelParameters(1,100));
    _default_configs.insert(make_pair("Catafron", params));


    params.clear();
    params.push_back(LevelParameters(4,3));
    params.push_back(LevelParameters(2,3));
    _default_configs.insert(make_pair("2Levels", params));

    params.clear();
    params.push_back(LevelParameters(1,10));
    _default_configs.insert(make_pair("1Level", params));

    params.clear();
    params.push_back(LevelParameters(4,3));
    params.push_back(LevelParameters(2,3));
    params.push_back(LevelParameters(1,3));
    _default_configs.insert(make_pair("Xtion160x120", params));

    params.clear();
    params.push_back(LevelParameters(16,3));
    params.push_back(LevelParameters(8,3));
    params.push_back(LevelParameters(4,3));
    params.push_back(LevelParameters(2,3));
    params.push_back(LevelParameters(1,3));
    _default_configs.insert(make_pair("ManyManyMany", params));

  }

  std::vector<std::string> ProjectiveAligner::defaultConfigs(){
    std::vector<std::string> v;
    v.resize(_default_configs.size());
    int i;
    for (std::map<std::string, std::vector<ProjectiveAligner::LevelParameters> >::iterator it = _default_configs.begin();
	 it!=_default_configs.end(); it++){
      v[i] = it->first;
    }
    return v;
  }


  void ProjectiveAligner::setDefaultConfig(const std::string config){
    std::map<std::string, std::vector<ProjectiveAligner::LevelParameters> >::iterator it = _default_configs.find(config);
    if (it==_default_configs.end())
      throw std::runtime_error("ProjectiveAlinger::setDefaultConfig, config requested does not exist");
    _level_parameters = it->second;
  }

  ProjectiveAligner::ProjectiveAligner(BaseProjector* p):
    _projector(p ? p : new PinholeProjector()),
    _finder(&_solver, _projector )
  {
    _initDefaultConfigs();
    _solver.setMaxError(.0001);
    _solver.setDamping(100);
    _solver.setGICP(true);
    setMaxDistance(3);
    _finder.setPointsDistance(0.1);
    _reference_canvas_scale = 1.5f;
    setDefaultConfig("Xtion320x240");
  }

  ProjectiveAligner::~ProjectiveAligner() {}


  void ProjectiveAligner::align(const Eigen::Isometry3f& initial_guess,
				const Matrix6f& initial_guess_information){
    if (! _solver.referenceModel()) {
      throw std::runtime_error("ProjectiveAligner: align(), reference model not set");
    }
    if (!_solver.currentModel()){
      throw std::runtime_error("ProjectiveAligner: align(), current model not set");
    }

    if(! _level_parameters.size()){
      throw std::runtime_error("ProjectiveAligner: align(), no levels in the pool");
    }

    int rows = _projector->imageRows();
    int cols = _projector->imageCols();

#ifdef _USE_IMAGE_SHRINK
    // the higher scale is 1
    _projector->pushState();
    int hrows = rows*_reference_canvas_scale;
    int hcols = cols*_reference_canvas_scale;
    _projector->setImageSize(hrows,hcols);
    _projector->scaleCamera(1);
    _projector->project(_reference_buffer_hres,
			_reference_indices_hres,
			Eigen::Isometry3f::Identity(),
			*_solver.referenceModel());
    // _projector->popState();

    // _projector->pushState();
    // _projector->setImageSize(rows,cols);
    // _projector->scaleCamera(1);
    _projector->project(_current_buffer_hres,
    			_current_indices_hres,
    			Eigen::Isometry3f::Identity(),
    			*_solver.currentModel());
    _projector->popState();

    std::vector<int> potentialReferenceIndices(hrows*hcols);
    size_t k = 0;
    for (int r = 0; r<_reference_indices_hres.rows; r++){
      for (int c = 0; c<_reference_indices_hres.cols; c++){
	int idx = _reference_indices_hres.at<int>(r,c);
	if (idx<0)
	  continue;
	potentialReferenceIndices[k] = idx;
	k++;
      }
    }
    potentialReferenceIndices.resize(k);
    _solver.setReferencePointsHint(potentialReferenceIndices);


#endif //_USE_IMAGE_SHRINK

    Matrix6f info;
    info.setIdentity();
    info *=10;

    if (initial_guess_information==Matrix6f::Zero()) 
      _solver.setT(initial_guess);
    else  {
      _solver.setT(initial_guess,initial_guess_information);
    }
    for (size_t g = 0; g< _level_parameters.size(); g++) {
      int scale = _level_parameters[g].scale;
      float registrationGain = 1./scale;
      
      //save the state of the projector
      _projector->pushState();
      
      _projector->setImageSize(_reference_canvas_scale*registrationGain*rows, 
			       _reference_canvas_scale*registrationGain*cols);
      _projector->scaleCamera(registrationGain);

#ifdef _USE_IMAGE_SHRINK
      FloatImage scaled_ref_buffer, scaled_curr_buffer;
      IntImage   scaled_ref_indices, scaled_curr_indices;
      shrinkDepth(scaled_ref_buffer, scaled_ref_indices, 
		  _reference_buffer_hres, _reference_indices_hres, scale);
      _finder.init(scaled_ref_buffer, scaled_ref_indices, _current_buffer_hres, _current_indices_hres, scale);
#else
      _finder.init();
      _solver.setReferencePointsHint(_finder.potentialReferenceIndices());
#endif //_USE_IMAGE_SHRINK
 
      //cerr << " time: " << tCorr << endl;
      for (int i = 0; i< _level_parameters[g].iterations; i++){
	_finder.compute();
	const BaseCorrespondenceFinder::CorrespondenceVector& corr=_finder.correspondences();
	
	// this is true at the last iteration of the last level 
	bool computeStats = (i == _level_parameters[g].iterations-1) && (g==_level_parameters.size()-1 );
	_solver.oneRound(corr, computeStats);
      }

      //restore the k and rhe image size in the projector
      _projector->popState();
    }
    
  }

}
