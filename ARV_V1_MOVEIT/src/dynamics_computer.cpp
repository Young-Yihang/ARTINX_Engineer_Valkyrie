
#include "dynamics_computer.hpp"
#include <cmath>  // For std::isfinite
#include <iostream>  // For std::cerr

DynamicsComputer::DynamicsComputer(const KDL::Chain &chain,
                                   const KDL::Vector &gravity)
    : gravity_(gravity)
{ // 创建KDL求解器
    dyn_param_ = std::make_unique<KDL::ChainDynParam>(chain, gravity_);
}

//前馈力矩，就是所谓系统分析得出的动力学力矩
void DynamicsComputer::computeFeedforwardTorque(
    const KDL::JntArray &q, 
    const KDL::JntArray &qd,
    const KDL::JntArray &qdd,
    KDL::JntArray &tau_ff)
{
    size_t n = q.rows(); // 关节数

    // 1. 计算惯性矩阵 M(q)
    KDL::JntSpaceInertiaMatrix M(n);
    dyn_param_->JntToMass(q, M);

    // 2. 计算科氏力和离心力 C(q, q̇)
    KDL::JntArray C(n);
    dyn_param_->JntToCoriolis(q, qd, C);

    // 3. 计算重力项 G(q)
    KDL::JntArray G(n);
    dyn_param_->JntToGravity(q, G);

    // 4. 计算前馈力矩: τ_ff = M·q̈ + C + G
    for (size_t i = 0; i < n; i++)
    {
        tau_ff(i) = 0.0;

        // M·q̈ (矩阵乘法)
        for (size_t j = 0; j < n; j++)
        {
            tau_ff(i) += M(i, j) * qdd(j);
        }

        // 加上 C 和 G
        tau_ff(i) += C(i) + G(i);

        // ========== SAFETY: Check for NaN/Inf in dynamics computation ==========
        if (!std::isfinite(tau_ff(i)))
        {
            // Log error but return zero torque as safe fallback
            // Note: We use std::cerr here as we don't have access to ROS logger
            std::cerr << "[SAFETY] Dynamics solver produced non-finite torque on joint "
                      << i << " (M*qdd + C + G = NaN/Inf), returning zero" << std::endl;
            std::cerr << "  Inputs: q[" << i << "]=" << q(i)
                      << ", qd[" << i << "]=" << qd(i)
                      << ", qdd[" << i << "]=" << qdd(i) << std::endl;
            std::cerr << "  Components: C[" << i << "]=" << C(i)
                      << ", G[" << i << "]=" << G(i) << std::endl;
            tau_ff(i) = 0.0;  // Safe fallback
        }
    }
}

void DynamicsComputer::computeGravityTorque(
    const KDL::JntArray &q,
    KDL::JntArray &tau_ff)
{
    // 直接计算重力项 G(q)
    dyn_param_->JntToGravity(q, tau_ff);

    // ========== SAFETY: Check for NaN/Inf in gravity computation ==========
    for (size_t i = 0; i < q.rows(); i++)
    {
        if (!std::isfinite(tau_ff(i)))
        {
            std::cerr << "[SAFETY] Gravity computation produced non-finite torque on joint "
                      << i << ", returning zero" << std::endl;
            tau_ff(i) = 0.0;  // Safe fallback
        }
    }
}

void DynamicsComputer::getMassMatrix(const KDL::JntArray& q, 
                                     KDL::JntSpaceInertiaMatrix& M)
{
    dyn_param_->JntToMass(q, M);
}

void DynamicsComputer::getCoriolisForces(const KDL::JntArray& q, 
                                        const KDL::JntArray& qd,
                                        KDL::JntArray& C)
{
    dyn_param_->JntToCoriolis(q, qd, C);
}

void DynamicsComputer::getGravityForces(const KDL::JntArray& q,
                                       KDL::JntArray& G)
{
    dyn_param_->JntToGravity(q, G);
}
