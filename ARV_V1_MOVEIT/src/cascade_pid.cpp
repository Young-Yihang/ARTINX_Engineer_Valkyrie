#include "cascade_pid.hpp"

// CascadePid: 外环P(位置)→内环PI(速度)→力矩
CascadePid::CascadePid(const PidGains& pos_gains,
                       const PidGains& vel_gains,
                       double max_vel,
                       double max_integral_pos,
                       double max_integral_vel)
    : pos_gains_(pos_gains),
      vel_gains_(vel_gains),
      max_vel_(max_vel),
      max_integral_pos_(max_integral_pos),
      max_integral_vel_(max_integral_vel),
      pos_error_(0.0),
      pos_error_prev_(0.0),
      pos_integral_(0.0),
      vel_error_(0.0),
      vel_error_prev_(0.0),
      vel_integral_(0.0),
      ref_vel_(0.0)
{
}

double CascadePid::compute(double pos_ref, double pos_fdb, double vel_fdb, double dt)
{
    // ========== 外环: 位置PID -> 期望速度 ==========
    
    // 1. 计算位置误差
    pos_error_ = pos_ref - pos_fdb;

    // 2. 位置比例项
    double pos_p = pos_gains_.kp * pos_error_;

    // 3. 位置积分项 (条件积分抗饱和)
    //    只有在误差较小时才积分,防止大误差时积分饱和
    const double integral_threshold = 0.1;  // 0.1 rad ≈ 5.7°
    if (std::abs(pos_error_) < integral_threshold)
    {
        pos_integral_ += pos_error_ * dt;
        // 积分限幅
        pos_integral_ = clamp(pos_integral_, -max_integral_pos_, max_integral_pos_);
    }
    double pos_i = pos_gains_.ki * pos_integral_;

    // 4. 位置微分项 (基于误差的微分)
    double pos_d = 0.0;
    if (dt > 1e-6)  // 避免除零
    {
        double pos_error_derivative = (pos_error_ - pos_error_prev_) / dt;
        pos_d = pos_gains_.kd * pos_error_derivative;
    }

    // 5. 外环输出 = 期望速度（反馈值）
    double ref_vel_fb = pos_p + pos_i + pos_d;

    // 6. 速度饱和保护
    ref_vel_fb = clamp(ref_vel_fb, -max_vel_, max_vel_);
    
    // 7. 最终期望速度 = 反馈值（无前馈时）
    ref_vel_ = ref_vel_fb;

    // 8. 更新历史误差
    pos_error_prev_ = pos_error_;

    // ========== 内环: 速度PID -> 控制力矩 ==========
    
    // 1. 计算速度误差
    vel_error_ = ref_vel_ - vel_fdb;

    // 2. 速度比例项
    double vel_p = vel_gains_.kp * vel_error_;

    // 3. 速度积分项 (条件积分抗饱和)
    const double vel_integral_threshold = 0.5;  // 0.5 rad/s
    if (std::abs(vel_error_) < vel_integral_threshold)
    {
        vel_integral_ += vel_error_ * dt;
        // 积分限幅
        vel_integral_ = clamp(vel_integral_, -max_integral_vel_, max_integral_vel_);
    }
    double vel_i = vel_gains_.ki * vel_integral_;

    // 4. 速度微分项 (基于误差的微分)
    double vel_d = 0.0;
    if (dt > 1e-6)  // 避免除零
    {
        double vel_error_derivative = (vel_error_ - vel_error_prev_) / dt;
        vel_d = vel_gains_.kd * vel_error_derivative;
    }

    // 5. 内环输出 = 控制力矩
    double torque = vel_p + vel_i + vel_d;

    // 6. 更新历史误差
    vel_error_prev_ = vel_error_;

    return torque;
}

void CascadePid::reset()
{
    pos_error_ = 0.0;
    pos_error_prev_ = 0.0;
    pos_integral_ = 0.0;

    vel_error_ = 0.0;
    vel_error_prev_ = 0.0;
    vel_integral_ = 0.0;

    ref_vel_ = 0.0;
}

void CascadePid::setPositionGains(const PidGains& gains)
{
    pos_gains_ = gains;
}

void CascadePid::setVelocityGains(const PidGains& gains)
{
    vel_gains_ = gains;
}

void CascadePid::setMaxVelocity(double max_vel)
{
    max_vel_ = max_vel;
}

// MultiJointCascadePid: 6关节级联PID管理器
MultiJointCascadePid::MultiJointCascadePid(size_t num_joints)
{
    // 为每个关节创建一个级联PID控制器
    // 初始参数为零,后续通过setJointParams()设置
    PidGains default_gains(0.0, 0.0, 0.0);
    for (size_t i = 0; i < num_joints; ++i)
    {
        controllers_.emplace_back(default_gains, default_gains, 1.0);
    }
}

void MultiJointCascadePid::setJointParams(size_t joint_idx,
                                          const PidGains& pos_gains,
                                          const PidGains& vel_gains,
                                          double max_vel,
                                          double max_integral_pos,
                                          double max_integral_vel)
{
    if (joint_idx >= controllers_.size())
    {
        return;  // 索引越界保护
    }

    // 重新构造该关节的控制器
    controllers_[joint_idx] = CascadePid(pos_gains, vel_gains, max_vel,
                                         max_integral_pos, max_integral_vel);
}

void MultiJointCascadePid::compute(const std::vector<double>& pos_ref,
                                   const std::vector<double>& pos_fdb,
                                   const std::vector<double>& vel_fdb,
                                   double dt,
                                   std::vector<double>& torque_out)
{
    // 确保输出向量大小正确
    torque_out.resize(controllers_.size());

    // 对每个关节分别计算
    for (size_t i = 0; i < controllers_.size(); ++i)
    {
        torque_out[i] = controllers_[i].compute(pos_ref[i], pos_fdb[i], vel_fdb[i], dt);
    }
}

void MultiJointCascadePid::resetAll()
{
    for (auto& controller : controllers_)
    {
        controller.reset();
    }
}

CascadePid& MultiJointCascadePid::getJointController(size_t joint_idx)
{
    return controllers_[joint_idx];
}
