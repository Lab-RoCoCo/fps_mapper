#include "globals/defs.h"
#include <iostream>

using namespace std;

struct Plane{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Plane() {
    _normal << 0,0,1;
    _d =0;
  }

  Plane(const Eigen::Vector3f& n, float& d_) {
    _normal=n;
    _d =d_;
  }

  inline const Eigen::Vector3f& normal() const {return _normal;}
  inline void setNormal(const Eigen::Vector3f& n)  { _normal = n;}

  inline float d() const {return _d;}
  inline void setD(float d_)  { _d = d_;}

  inline Plane transform(const Eigen::Isometry3f& iso) const{
    Eigen::Vector3f tn=iso.linear()*_normal;
    float td = _d-tn.dot(iso.translation());
    return Plane(tn,td);
  }

  inline Eigen::Vector4f operator-(const Plane& p2) const{
    Eigen::Vector4f v;
    v.block<3,1>(0,0)=_normal - p2._normal;
    v(3)=_d - p2._d;
    return v;
  }

  Eigen::Vector3f _normal;
  float _d;
};


typedef std::vector<Plane, Eigen::aligned_allocator<Plane> > PlaneVector;

typedef Eigen::Matrix<float, 4, 6> Matrix4_6f;

struct PlaneMeasurement{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlaneMeasurement(const Plane& r, const Plane& c){
    _reference = r;
    _current = c;
  }
  
  Eigen::Vector4f error(const Eigen::Isometry3f& iso) const {
    return _reference - _current.transform(iso);
  }

  Matrix4_6f Jacobian(const Eigen::Isometry3f& iso) const {
    Matrix4_6f J;
    J.setZero();
    float epsilon = 1e-3;
    float i_epsilon = 1./epsilon;
    for (int i =0; i<6; i++){
      Vector6f v_plus = Vector6f::Zero();
      Vector6f v_minus = Vector6f::Zero();
      v_plus[i]=epsilon;
      v_minus[i]=-epsilon;
      J.col(i)=(error(v2t(v_plus)*iso)-error(v2t(v_minus)*iso))*i_epsilon;
    }
    return J;
  }

  
  Plane _reference;
  Plane _current;
};

typedef std::vector<PlaneMeasurement, Eigen::aligned_allocator<PlaneMeasurement> > PlaneMeasurementVector;

struct PlaneSolver {
  PlaneSolver(){
    _T.setIdentity();
    _damping = 10;
  }

  const Eigen::Isometry3f& T() const {return _T;}
  void setT(const Eigen::Isometry3f& T_)  {_T = T_;}
  
  void setDamping(float d) { _damping = d; }

  float damping() const {return _damping;}


  const PlaneMeasurementVector& measurements() const {return _measurements;}

  PlaneMeasurementVector& measurements() {return _measurements;}
  
  float oneRound(){
    Matrix6f H = Matrix6f::Zero();
    Vector6f b = Vector6f::Zero();
    float error = 0;
    for (size_t i = 0; i<_measurements.size(); i++){
      const PlaneMeasurement& measurement = _measurements[i];
      Eigen::Vector4f e=measurement.error(_T);
      Matrix4_6f J = measurement.Jacobian(_T);
      H+=J.transpose()*J;
      b+=J.transpose()*e;
      error += e.dot(e);
    }
    H+=Matrix6f::Identity()*_damping;
    Vector6f dt = H.ldlt().solve(-b);
    _T=v2t(dt)*_T;
    // recondition rotation afterm many multiplications
    Eigen::Matrix3f R = _T.linear();
    Eigen::Matrix3f E = R.transpose() * R;
    E.diagonal().array() -= 1;
    _T.linear() -= 0.5 * R * E;
    return error;
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlaneMeasurementVector _measurements;
  Eigen::Isometry3f _T;
  float _damping;

};


int main(int argc, char** argv) {
  // generate a random cloud of planes;
  PlaneVector planes(300);
  for (size_t i = 0; i<planes.size(); i++){
    Eigen::Vector3f n(drand48()-.5, drand48()-.5, drand48()-.5);
    n.normalize();
    float d = 10*(drand48()-.5);
    planes[i]=Plane(n,d);
  }

  Vector6f tv;
  tv << 10, 20, 30, .2, -.3, .4;

  Eigen::Isometry3f T=v2t(tv);
  Eigen::Isometry3f iT=T.inverse();
  

  // generate the measurements, by rotating the planes according to the fake transform found
  PlaneMeasurementVector measurements;
  for (size_t i=0; i<planes.size(); i++) {
    Plane seen_by_laser=planes[i];
    Plane seen_by_kinect=seen_by_laser.transform(iT);
    measurements.push_back(PlaneMeasurement(seen_by_laser, seen_by_kinect));
  }

  // call the solver
  PlaneSolver solver;
  solver.measurements() = measurements;
  int iterations = 30;
  for (int i = 0; i<iterations; i++){
    cerr << "Iteration: " << i ;
    float error = solver.oneRound();
    cerr << " error: " << error;

    cerr << " T: " << t2v(solver.T()).transpose();
    cerr << endl;

  }


}
