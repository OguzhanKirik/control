#pragma once
// Minimal dynamics stubs for UR5: M(q), C(q, qd), G(q)
// Header-only. By default returns Identity/Zero so you can compile & test quickly.
// Replace with a real model (e.g., from KDL/RBDL/Pinocchio) when ready.

#include <Eigen/Dense>

namespace ctrl {

using Eigen::VectorXd;
using Eigen::MatrixXd;

// --- STUB MODEL (safe defaults) ---------------------------------------------

// Inertia matrix M(q) ~ I
inline MatrixXd M_of(const VectorXd& q)
{
  (void)q;
  MatrixXd M = MatrixXd::Identity(6,6);
  return M;
}

// Coriolis/centrifugal term C(q, qd) ~ 0
inline VectorXd C_of(const VectorXd& q, const VectorXd& qd)
{
  (void)q; (void)qd;
  return VectorXd::Zero(6);
}

// Gravity torque G(q) ~ 0   (replace with real gravity if you have it)
inline VectorXd G_of(const VectorXd& q)
{
  (void)q;
  return VectorXd::Zero(6);
}

// ---------------------------------------------------------------------------
// If you want a quick-and-dirty gravity approximation, you could add a simple
// constant torque bias per joint here. For proper results, use a library:
//  - Orocos KDL: build KDL::Chain from URDF, then ChainDynParam::JntToMass / Coriolis / Gravity
//  - RBDL / Pinocchio: load URDF and call ABA/CRBA algorithms
//
// Example signatures you might implement later (KDL-backed):
//
// MatrixXd M_kdl(const KDL::Chain& chain, const KDL::JntArray& q);
// VectorXd C_kdl(const KDL::Chain& chain, const KDL::JntArray& q, const KDL::JntArray& qd);
// VectorXd G_kdl(const KDL::Chain& chain, const KDL::JntArray& q);
// ---------------------------------------------------------------------------

} // namespace ctrl
