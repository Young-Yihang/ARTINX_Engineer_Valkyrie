#ifndef DYNAMICS_COMPUTER_HPP
#define DYNAMICS_COMPUTER_HPP

#include <memory>
#include <functional>
#include <string>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>

class DynamicsComputer
{
public:
    // 日志回调函数类型定义
    using LogCallback = std::function<void(const std::string &)>;

    DynamicsComputer(const KDL::Chain &chain, // 构造函数，传入重力向量和KDL运动链
                     const KDL::Vector &gravity = KDL::Vector(0.0, 0.0, -9.81));

    // 设置错误日志回调函数
    void setErrorLogger(LogCallback callback) { error_logger_ = callback; }

    /**
     * @brief 计算前馈力矩 τ_ff = M(q)q̈ + C(q,q̇) + G(q)
     */
    void computeFeedforwardTorque(
        const KDL::JntArray &q,
        const KDL::JntArray &qd,
        const KDL::JntArray &qdd,
        KDL::JntArray &tau_ff);

    /**
     * @brief 计算重力补偿力矩 τ_g = G(q)
     * @param q 期望关节位置
     * @param tau_ff 输出：前馈力矩
     */
    void computeGravityTorque(
        const KDL::JntArray &q,
        KDL::JntArray &tau_ff);

    /**
     * @brief 单独获取惯性矩阵 M(q)
     */
    void getMassMatrix(const KDL::JntArray &q,
                       KDL::JntSpaceInertiaMatrix &M);

    /**
     * @brief 单独获取科氏力和离心力 C(q, q̇)
     */
    void getCoriolisForces(const KDL::JntArray &q,
                           const KDL::JntArray &qd,
                           KDL::JntArray &C);

    /**
     * @brief 单独获取重力项 G(q)
     */
    void getGravityForces(const KDL::JntArray &q,
                          KDL::JntArray &G);

private:
    std::unique_ptr<KDL::ChainDynParam> dyn_param_; // KDL智能指针，指向动力学求解
    KDL::Vector gravity_;
    LogCallback error_logger_; // 错误日志回调函数
};

#endif