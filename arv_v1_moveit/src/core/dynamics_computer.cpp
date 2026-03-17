/// @file dynamics_computer.cpp
/// @brief KDL rigid-body dynamics with NaN/Inf safety.
#include "dynamics_computer.hpp"

#include <cmath>    // For std::isfinite
#include <sstream>  // For std::ostringstream

DynamicsComputer::DynamicsComputer(const KDL::Chain &chain, const KDL::Vector &gravity)
    : gravity_(gravity) {
  dyn_param_ = std::make_unique<KDL::ChainDynParam>(chain, gravity_);
}

// τ_ff = M(q)·q̈ + C(q,q̇) + G(q)
void DynamicsComputer::computeFeedforwardTorque(const KDL::JntArray &q, const KDL::JntArray &qd,
                                                const KDL::JntArray &qdd, KDL::JntArray &tau_ff) {
  size_t n = q.rows();

  KDL::JntSpaceInertiaMatrix M(n);
  dyn_param_->JntToMass(q, M);
  KDL::JntArray C(n);
  dyn_param_->JntToCoriolis(q, qd, C);
  KDL::JntArray G(n);
  dyn_param_->JntToGravity(q, G);

  for (size_t i = 0; i < n; i++) {
    tau_ff(i) = 0.0;
    for (size_t j = 0; j < n; j++) {
      tau_ff(i) += M(i, j) * qdd(j);
    }

    tau_ff(i) += C(i) + G(i);

    // --- SAFETY: Check for NaN/Inf in dynamics computation ---
    if (!std::isfinite(tau_ff(i))) {
      if (error_logger_) {
        std::ostringstream oss;
        oss << "[SAFETY] Dynamics solver produced non-finite torque on joint " << i
            << " (M*qdd + C + G = NaN/Inf), returning zero. " << "Inputs: q[" << i << "]=" << q(i)
            << ", qd[" << i << "]=" << qd(i) << ", qdd[" << i << "]=" << qdd(i)
            << ". Components: C[" << i << "]=" << C(i) << ", G[" << i << "]=" << G(i);
        error_logger_(oss.str());
      }
      tau_ff(i) = 0.0;  // Safe fallback
    }
  }
}

void DynamicsComputer::computeGravityTorque(const KDL::JntArray &q, KDL::JntArray &tau_ff) {
  dyn_param_->JntToGravity(q, tau_ff);

  // --- SAFETY: Check for NaN/Inf in gravity computation ---
  for (size_t i = 0; i < q.rows(); i++) {
    if (!std::isfinite(tau_ff(i))) {
      if (error_logger_) {
        std::ostringstream oss;
        oss << "[SAFETY] Gravity computation produced non-finite torque on joint " << i
            << ", returning zero";
        error_logger_(oss.str());
      }
      tau_ff(i) = 0.0;  // Safe fallback
    }
  }
}

void DynamicsComputer::getMassMatrix(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix &M) {
  dyn_param_->JntToMass(q, M);
}

void DynamicsComputer::getCoriolisForces(const KDL::JntArray &q, const KDL::JntArray &qd,
                                         KDL::JntArray &C) {
  dyn_param_->JntToCoriolis(q, qd, C);
}

void DynamicsComputer::getGravityForces(const KDL::JntArray &q, KDL::JntArray &G) {
  dyn_param_->JntToGravity(q, G);
}
