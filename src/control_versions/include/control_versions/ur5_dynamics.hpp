// ur5_dynamics.hpp
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cmath>

using Eigen::Matrix3d; using Eigen::Matrix4d;
using Eigen::Vector3d; using Eigen::VectorXd; using Eigen::MatrixXd;

struct DHRow { double a, alpha, d, theta0; };

// ==== UR5 DH (modify if your convention differs; this is standard DH) ====
static const std::array<DHRow,6> DH_UR5 = {{
  {  0.0,            M_PI/2, 0.089159, 0.0},
  { -0.425,          0.0,    0.0,      0.0},
  { -0.39225,        0.0,    0.0,      0.0},
  {  0.0,            M_PI/2, 0.10915,  0.0},
  {  0.0,           -M_PI/2, 0.09465,  0.0},
  {  0.0,            0.0,    0.0823,   0.0}
}};

// ==== UR5 inertial placeholders (replace with identified data) ====
// Each link i has: mass m_i, COM in local frame r_i, inertia about COM Ic_i (3x3, link frame)
struct LinkInertia {
  double m;
  Vector3d r;      // COM position in link frame {i}
  Matrix3d Ic;     // inertia about COM, expressed in {i}
};
static const std::array<LinkInertia,6> UR5_INERTIA_APPROX = {{
  {3.5,  Vector3d(0.0, -0.025,  0.10), (Matrix3d() << 0.014,0,0, 0,0.018,0, 0,0,0.012).finished()},
  {8.0,  Vector3d(0.0,  0.000,  0.25), (Matrix3d() << 0.20,0,0,  0,0.19,0,  0,0,0.02).finished()},
  {2.8,  Vector3d(0.0,  0.000,  0.20), (Matrix3d() << 0.05,0,0,  0,0.05,0,  0,0,0.01).finished()},
  {1.2,  Vector3d(0.0,  0.000,  0.05), (Matrix3d() << 0.005,0,0,0,0.005,0,0,0,0.004).finished()},
  {1.0,  Vector3d(0.0,  0.000,  0.02), (Matrix3d() << 0.003,0,0,0,0.003,0,0,0,0.002).finished()},
  {0.5,  Vector3d(0.0,  0.000,  0.01), (Matrix3d() << 0.001,0,0,0,0.001,0,0,0,0.001).finished()}
}};

// ==== Helpers ====
inline Matrix4d dhT(double a,double alpha,double d,double theta){
  double ca=std::cos(alpha), sa=std::sin(alpha);
  double ct=std::cos(theta), st=std::sin(theta);
  Matrix4d T;
  T<< ct,-st*ca, st*sa, a*ct,
      st, ct*ca,-ct*sa, a*st,
       0,    sa,    ca,    d,
       0,     0,     0,    1;
  return T;
}
inline Matrix3d skew(const Vector3d& v){
  Matrix3d S; S<< 0,-v.z(), v.y(), v.z(),0,-v.x(), -v.y(), v.x(),0; return S;
}

// ==== Newton–Euler inverse dynamics (RNEA), revolute Z joints, standard DH ====
struct NEContext {
  std::array<Matrix3d,7> R;     // rotation ^{i-1}R_i
  std::array<Vector3d,7> p;     // vector from {i-1} to {i} origin, expressed in {i-1}
  std::array<Vector3d,7> z;     // joint axis in each frame (z_i)
};

inline NEContext build_kin_chain(const VectorXd& q){
  NEContext K;
  for(int i=0;i<6;++i){
    const auto& r = DH_UR5[i];
    double th = r.theta0 + q(i);
    Matrix4d A = dhT(r.a, r.alpha, r.d, th);
    K.R[i+1] = A.block<3,3>(0,0);
    K.p[i+1] = A.block<3,1>(0,3);
    K.z[i+1] = Vector3d(0,0,1); // z-axis of frame i (in its own frame)
  }
  return K;
}

// gravity in base (world) frame; we propagate it down
inline Vector3d gravity_base(){ return Vector3d(0,0,-9.81); }

// Main RNEA: tau = ID(q,qd,qdd)
inline VectorXd inverse_dynamics(const VectorXd& q,
                                 const VectorXd& qd,
                                 const VectorXd& qdd,
                                 bool include_gravity = true)
{
  auto K = build_kin_chain(q);

  // Forward recursion (express EVERYTHING in local frame i)
  std::array<Vector3d,7> w{}, wdot{}, vdot{};   // angular vel/acc, linear acc at origin
  w[0].setZero(); wdot[0].setZero();
  if (include_gravity) {
    vdot[0] = -K.R[1]*gravity_base(); // push g into chain first link approx
  } else {
    vdot[0] = Vector3d::Zero();
  }

  for(int i=1;i<=6;++i){
    const Matrix3d& R = K.R[i];           // ^{i-1}R_i
    const Vector3d& p = K.p[i];           // ^{i-1}p_i
    const Vector3d  z = Vector3d(0,0,1);  // z in frame i

    // Rotate previous to current frame
    Vector3d w_im1   = R.transpose() * w[i-1];
    Vector3d wdot_im1= R.transpose() * wdot[i-1];
    Vector3d vdot_im1= R.transpose() * ( vdot[i-1]
                         + w[i-1].cross( w[i-1].cross(p) ) + wdot[i-1].cross(p) );

    // Joint motion (revolute about z)
    w[i]    = w_im1    + z * qd(i-1);
    wdot[i] = wdot_im1 + z * qdd(i-1) + w[i].cross( z * qd(i-1) );
    vdot[i] = vdot_im1; // origin acceleration already handled
  }

  // Backward recursion
  std::array<Vector3d,7> f{}, n{}; // spatial force: linear f and moment n in frame i
  for(int i=6;i>=1;--i){
    const auto& I  = UR5_INERTIA_APPROX[i-1];
    const Vector3d& rc = I.r;           // COM in frame i
    const Matrix3d& Ic = I.Ic;          // inertia at COM
    // COM acceleration
    Vector3d a_c = vdot[i] + wdot[i].cross(rc) + w[i].cross( w[i].cross(rc) );

    // Link forces
    Vector3d F = I.m * a_c;
    Vector3d N = Ic*wdot[i] + w[i].cross(Ic*w[i]) + rc.cross(F);

    // accumulate children (only one child per chain here)
    if(i<6){
      const Matrix3d& R_ip1 = K.R[i+1];      // ^{i}R_{i+1}
      const Vector3d& p_ip1 = K.p[i+1];      // ^{i}p_{i+1}
      // bring child forces to frame i
      Vector3d f_child = R_ip1 * f[i+1];
      Vector3d n_child = R_ip1 * n[i+1] + p_ip1.cross(f_child);
      F += f_child; N += n_child;
    }
    f[i]=F; n[i]=N;
  }

  // Joint torques: project moment on joint axis z_i
  VectorXd tau(6); tau.setZero();
  for(int i=1;i<=6;++i){
    Vector3d z = Vector3d(0,0,1);
    tau(i-1) = n[i].dot(z); // revolute about z
  }
  return tau;
}

// Build M(q) by exciting unit accelerations (qd=0, g=0)
inline MatrixXd mass_matrix(const VectorXd& q){
  MatrixXd M(6,6); M.setZero();
  VectorXd qd=VectorXd::Zero(6), qdd=VectorXd::Zero(6);
  for(int j=0;j<6;++j){
    qdd.setZero(); qdd(j)=1.0;
    VectorXd tau = inverse_dynamics(q, qd, qdd, /*include_gravity=*/false);
    M.col(j)=tau;
  }
  return M;
}

// Nonlinear term h(q,qd) = C(q,qd)qd + g(q)
inline VectorXd nonlinear_term(const VectorXd& q, const VectorXd& qd){
  VectorXd tau_total = inverse_dynamics(q, qd, VectorXd::Zero(6), /*include_gravity=*/true);
  VectorXd tau_g     = inverse_dynamics(q, VectorXd::Zero(6), VectorXd::Zero(6), /*gravity=*/true);
  return tau_total - tau_g;
}
