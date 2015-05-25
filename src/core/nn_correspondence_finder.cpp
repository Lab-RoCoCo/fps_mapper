#include "nn_correspondence_finder.h"

namespace fps_mapper {
  NNCorrespondenceFinder::NNCorrespondenceFinder(Solver* s):
    BaseCorrespondenceFinder(s){
    _index = 0;
    _knn = 1;
    _normal_scaling = 1;
  }

  NNCorrespondenceFinder::~NNCorrespondenceFinder() {
    if (_index)
      delete _index;
  }

  void model2linear(std::vector<float>& dest, const Cloud& src, float nscale){
    int k=0;
    dest.resize(src.size()*6); // relies on the packetization of the data
    float* dp=&dest[0];
    for (size_t i=0; i<src.size(); i++){
      const Eigen::Vector3f& p = src[i].point();
      const Eigen::Vector3f n = src[i].normal()*nscale;
      *dp=p.x(); ++dp;
      *dp=p.y(); ++dp;
      *dp=p.z(); ++dp;
      *dp=n.x(); ++dp;
      *dp=n.y(); ++dp;
      *dp=n.z(); ++dp;
    }
  }

  void model2linear(std::vector<float>& dest, const Cloud& src, float nscale, const Eigen::Isometry3f& T){
    int k=0;
    dest.resize(src.size()*6); // relies on the packetization of the data
    float* dp=&dest[0];
    Eigen::Matrix3f sR=T.linear()*nscale;
    for (size_t i=0; i<src.size(); i++){
      Eigen::Vector3f p = T*src[i].point();
      Eigen::Vector3f n = sR*src[i].normal();
      *dp=p.x(); ++dp;
      *dp=p.y(); ++dp;
      *dp=p.z(); ++dp;
      *dp=n.x(); ++dp;
      *dp=n.y(); ++dp;
      *dp=n.z(); ++dp;
    }
  }

  void NNCorrespondenceFinder::init(){
    if (! _solver->referenceModel()){
      throw std::runtime_error("NNCorrespondenceFinder::init(), no reference model in solver");
    }
    const Cloud& ref = *_solver->referenceModel();
    model2linear(_reference_points, ref, _normal_scaling);
    if (_index)
      delete(_index);
    _index = 0;
    
    _reference_matrix = flann::Matrix<float> (&_reference_points[0], ref.size(), 6);
    _index = new flann::Index< flann::L2<float> >(_reference_matrix, flann::KDTreeIndexParams(4));
    //cerr << "building index ... ";
    _index->buildIndex();
    //cerr << " done" << endl;
    
    if (! _solver->currentModel()){
      throw std::runtime_error("NNCorrespondenceFinder::init(), no reference model in solver");
    }

    // allocate the workspace for the current model and for the search
    const Cloud& curr = *_solver->currentModel();
    model2linear(_current_points, curr, _normal_scaling);
    _current_matrix = flann::Matrix<float> (&_current_points[0], curr.size(), 6);
    _current_indices.resize(curr.size()*_knn);
    _indices_matrix = flann::Matrix<int>(&_current_indices[0], curr.size(), _knn);
    _current_distances.resize(curr.size()*_knn);
    _distances_matrix = flann::Matrix<float>(&_current_distances[0], curr.size(), _knn);
    _correspondences.reserve(_indices_matrix.rows*_indices_matrix.cols);
  }

  void NNCorrespondenceFinder::compute(){
    const Cloud& curr = *_solver->currentModel();
    const Cloud& ref = *_solver->referenceModel();
    Eigen::Isometry3f T = _solver->T();
    float ndist = cos(_normal_angle);
    float pdist = _points_distance * _points_distance;

    model2linear(_current_points, curr, _normal_scaling, T);
    /*
    cerr << "doing search ... ";
    _index->knnSearch(_current_matrix, 
		      _indices_matrix, 
		      _distances_matrix, _knn, flann::SearchParams(16));
    cerr << " done" << endl;
    */

    flann::SearchParams params(16);
    params.cores = 8;
    //cerr << "doing search ... ";
    _index->radiusSearch(_current_matrix, 
			 _indices_matrix, 
			 _distances_matrix, pdist, flann::SearchParams(16));
    //cerr << " done" << endl;

    

    _correspondences.clear();
    // scan through the returned indices and compute the correspondences
    int k = 0;

    //cerr << "building correspondences ... " << _indices_matrix.rows << " " << _indices_matrix.cols;
    for(size_t cidx=0; cidx<_indices_matrix.rows; cidx++) {
      int* referenceIndexPtr = _indices_matrix.ptr()+(cidx*_knn);
      float* distancesPtr = _distances_matrix.ptr()+(cidx*_knn);
      Eigen::Vector3f cn = T.linear() * curr[cidx].normal();
      Eigen::Vector3f cp = T * curr[cidx].point();

      for (size_t j=0; j<_indices_matrix.cols; j++) {
	int ridx = *referenceIndexPtr;
	float d = *distancesPtr;
	referenceIndexPtr++;
	distancesPtr++;

	if (ridx < 0)
	  continue;
	const Eigen::Vector3f& rn = ref[ridx].normal();
	const Eigen::Vector3f& rp = ref[ridx].point();
	if ((cp-rp).squaredNorm()>pdist)
	  continue;

	if (cn.dot(rn)<ndist)
	  continue;
	_correspondences.push_back(std::make_pair(ridx, cidx));
      }
    }
    //cerr << "done " << endl;
    // cerr << "found " << _correspondences.size() << " correspondences" << endl;
  }

}
