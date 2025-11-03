#pragma once
// Minimal UR5 kinematics helpers: DH-based FK, geometric Jacobian, and SO(3) log error.
// Header-only, Eigen-only (no ROS/KDL dependency).

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cmath>
#include <iostream>

namespace ctrl {

using Eigen::Matrix4d;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::MatrixXd;

struct DHRow { double a, alpha, d, theta0; };

// UR5 DH parameters (UR classic, tool0 at flange)
static constexpr std::array<DHRow,6> DH_UR5 = {{
  {  0.0,            M_PI/2, 0.089159, 0.0},
  { -0.425,          0.0,    0.0,      0.0},
  { -0.39225,        0.0,    0.0,      0.0},
  {  0.0,            M_PI/2, 0.10915,  0.0},
  {  0.0,           -M_PI/2, 0.09465,  0.0},
  {  0.0,            0.0,    0.0823,   0.0}
}};

inline Matrix4d dhT(double a, double alpha, double d, double theta){
  const double ca = std::cos(alpha), sa = std::sin(alpha);
  const double ct = std::cos(theta), st = std::sin(theta);
  Matrix4d T;
  T <<  ct, -st*ca,  st*sa, a*ct,
        st,  ct*ca, -ct*sa, a*st,
         0,     sa,     ca,    d,
         0,      0,      0,    1;
  return T;
}

inline void rpy_from_R_zyx(const Matrix3d& R, double& phi, double& theta, double& psi){
  psi   = std::atan2(R(1,0), R(0,0));
  theta = std::asin(-R(2,0));
  phi   = std::atan2(R(2,1), R(2,2));
}

inline Matrix3d W_rpy_zyx(double phi, double theta){
  const double sφ = std::sin(phi),  cφ = std::cos(phi);
  const double tθ = std::tan(theta), cθ = std::cos(theta);
  Matrix3d W;
  W << 1,   sφ*tθ,   cφ*tθ,
       0,   cφ,     -sφ,
       0,   sφ/cθ,   cφ/cθ;
  return W;
}

// Forward kinematics for all frames (0..6). Outputs transforms, origins, z-axes.
inline void fk_all(const std::array<DHRow,6>& DH,
                   const Eigen::Matrix<double,6,1>& q,
                   std::array<Matrix4d,7>& T,
                   std::array<Vector3d,7>& o,
                   std::array<Vector3d,7>& z)
{
  T[0].setIdentity(); o[0].setZero(); z[0] = Vector3d(0,0,1);
  for(int i=0;i<6;++i){
    const auto& r = DH[i];
    const double th = r.theta0 + q(i);
    Matrix4d A = dhT(r.a, r.alpha, r.d, th);
    T[i+1] = T[i] * A;
    o[i+1] = T[i+1].block<3,1>(0,3);
    z[i+1] = T[i+1].block<3,1>(0,2);
  }
}

// Orientation error as rotation vector: log(R_des * R_cur^T)
inline Vector3d rotvec_error(const Matrix3d& R_cur, const Matrix3d& R_des){
  Matrix3d Rerr = R_des * R_cur.transpose();
  double tr = Rerr.trace();
  double c = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));
  double th = std::acos(c);
  if (th < 1e-9) return Vector3d::Zero();
  Matrix3d K = (Rerr - Rerr.transpose()) / (2.0*std::sin(th));
  Vector3d axis(K(2,1), K(0,2), K(1,0));
  return th * axis;
}

// FK (T06) and 6×6 geometric Jacobian Jg = [Jv; Jw] in base frame
inline void fk_and_jacobian(const std::array<DHRow,6>& DH,
                            const Eigen::Matrix<double,6,1>& q,
                            Matrix4d& T06,
                            MatrixXd& Jg /*6x6*/)
{
  std::array<Matrix4d,7> T;
  std::array<Vector3d,7> o;
  std::array<Vector3d,7> z;
  fk_all(DH, q, T, o, z);
  T06 = T[6];

  Jg.resize(6,6);
  Jg.setZero();
  const Vector3d pe = o[6];
  for(int i=0;i<6;++i){
    const Vector3d zi = z[i];
    const Vector3d oi = o[i];
    Vector3d Jv = zi.cross(pe - oi);
    Jg.block<3,1>(0,i) = Jv;
    Jg.block<3,1>(3,i) = zi;
  }

  // Optional: build analytical Ja by multiplying the rotational part with W(φ,θ)
  // Beware gimbal lock if |cos(theta)| ~ 0
  // double φ, θ, ψ; rpy_from_R_zyx(T06.block<3,3>(0,0), φ, θ, ψ);
  // if (std::abs(std::cos(θ)) < 1e-6) std::cerr << "[warn] ZYX near gimbal lock\n";
}

} // namespace ctrl
