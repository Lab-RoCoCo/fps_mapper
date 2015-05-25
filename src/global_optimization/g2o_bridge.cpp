#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/optimizable_graph.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/robust_kernel_impl.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o_bridge.h"
namespace fps_mapper {

  using namespace std;

  G2OBridge::G2OBridge() {
    graph = g2oInit();
  }

  G2OBridge::~G2OBridge() {
    delete graph;
  }

  void G2OBridge::optimize(int iterations_) {
    graph->initializeOptimization();
    graph->setVerbose(true);
    graph->optimize(iterations_);
  }

  void G2OBridge::quietOptimize(int iterations_) {
    graph->initializeOptimization();
    graph->setVerbose(false);
    graph->optimize(iterations_);
  }

  void G2OBridge::psToG2o(BinaryNodeRelationSet& relations,
			    MapNodeList& local_maps){
    graph->clear();
    _nodes_g2o_ps_map.clear();
    _edges_g2o_ps_map.clear();
    _nodes_ps_g2o_map.clear();
    _edges_ps_g2o_map.clear();
    int id=0;
    for (MapNodeList::iterator it=local_maps.begin(); it!=local_maps.end(); it++){
      g2o::VertexSE3* v=new g2o::VertexSE3;
      v->setId(id);
      v->setEstimate((*it)->transform().cast<double>());
      graph->addVertex(v);
      if (id==0)
	v->setFixed(true);
      _nodes_g2o_ps_map.insert(make_pair(v,it->get()));
      _nodes_ps_g2o_map.insert(make_pair(it->get(),v));
      id++;
    }
    for (BinaryNodeRelationSet::iterator it=relations.begin(); it!=relations.end(); it++){
      g2o::EdgeSE3* e=new g2o::EdgeSE3;
      const MapNode* n_from  = (*it)->from();
      const MapNode* n_to    = (*it)->to();
      g2o::VertexSE3* v_from = _nodes_ps_g2o_map[n_from];
      g2o::VertexSE3* v_to   = _nodes_ps_g2o_map[n_to];
      e->setVertex(0,v_from);
      e->setVertex(1,v_to);
      e->setMeasurement((*it)->transform().cast<double>());
      e->setInformation((*it)->informationMatrix().cast<double>());
      graph->addEdge(e);
      _edges_g2o_ps_map.insert(make_pair(e,it->get()));
      _edges_ps_g2o_map.insert(make_pair(it->get(),e));
    }
  }
  
  void G2OBridge::g2oToPs(MapNodeList& local_maps) {
    for (MapNodeList::iterator it=local_maps.begin(); it!=local_maps.end(); it++){
      g2o::VertexSE3* v=_nodes_ps_g2o_map[it->get()];
      (*it)->setTransform(v->estimate().cast<float>());
    }
  }

  g2o::SparseOptimizer * G2OBridge::g2oInit(){
    // graph construction
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef g2o::LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solverGauss   = new g2o::OptimizationAlgorithmLevenberg(blockSolver);
    //OptimizationAlgorithmGaussNewton* solverGauss   = new OptimizationAlgorithmGaussNewton(blockSolver);
    g2o::SparseOptimizer * g = new g2o::SparseOptimizer();
    g->setAlgorithm(solverGauss);
    return g;
  }
}
