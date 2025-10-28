
#include "dynamics_computer.hpp"

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
