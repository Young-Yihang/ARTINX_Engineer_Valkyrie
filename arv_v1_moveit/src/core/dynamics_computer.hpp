/// @file dynamics_computer.hpp
/// @brief KDL rigid-body dynamics: M(q), C(q,qd), G(q), feedforward torque.
#ifndef DYNAMICS_COMPUTER_HPP
#define DYNAMICS_COMPUTER_HPP

#include <functional>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <memory>
#include <string>

class DynamicsComputer {
public:
  using LogCallback = std::function<void(const std::string &)>;

  DynamicsComputer(const KDL::Chain &chain,
                   const KDL::Vector &gravity = KDL::Vector(0.0, 0.0, -9.81));

  void setErrorLogger(LogCallback callback) { error_logger_ = callback; }

  /// τ_ff = M(q)q̈ + C(q,q̇) + G(q)
  void computeFeedforwardTorque(const KDL::JntArray &q, const KDL::JntArray &qd,
                                const KDL::JntArray &qdd, KDL::JntArray &tau_ff);
  /// τ_g = G(q)
  void computeGravityTorque(const KDL::JntArray &q, KDL::JntArray &tau_ff);

  void getMassMatrix(const KDL::JntArray &q, KDL::JntSpaceInertiaMatrix &M);
  void getCoriolisForces(const KDL::JntArray &q, const KDL::JntArray &qd, KDL::JntArray &C);
  void getGravityForces(const KDL::JntArray &q, KDL::JntArray &G);

private:
  std::unique_ptr<KDL::ChainDynParam> dyn_param_;
  KDL::Vector gravity_;
  LogCallback error_logger_;
};

#endif