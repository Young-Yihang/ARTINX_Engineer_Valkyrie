/// @file cascade_pid.cpp
/// @brief Cascade PID: outer P(pos→vel) + inner PI(vel→torque)
#include "cascade_pid.hpp"

CascadePid::CascadePid(const PidGains &pos_gains, const PidGains &vel_gains, double max_vel,
                       double max_integral_pos, double max_integral_vel)
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
      ref_vel_(0.0), vel_cmd_filtered_(0.0), pos_fdb_prev_(0.0), vel_fdb_prev_(0.0) {}

double CascadePid::compute(double pos_ref, double pos_fdb, double vel_fdb, double dt) {
  // --- 外环: 位置PID → 期望速度 ---
  pos_error_ = pos_ref - pos_fdb;
  double pos_p = pos_gains_.kp * pos_error_;

  // 条件积分抗饱和: 仅小误差时积分
  const double integral_threshold = 0.1;  // 0.1 rad ≈ 5.7°
  if (std::abs(pos_error_) < integral_threshold) {
    pos_integral_ += pos_error_ * dt;
    pos_integral_ = clamp(pos_integral_, -max_integral_pos_, max_integral_pos_);
  }
  double pos_i = pos_gains_.ki * pos_integral_;

  double pos_d = 0.0;
  if (dt > 1e-6) {
    pos_d = -pos_gains_.kd * (pos_fdb - pos_fdb_prev_) / dt;
  }

  ref_vel_ = clamp(pos_p + pos_i + pos_d, -max_vel_, max_vel_);
  pos_error_prev_ = pos_error_;
  pos_fdb_prev_ = pos_fdb;

  // vel_cmd 一阶低通滤波（平滑 Kd 高频输出）
  vel_cmd_filtered_ = (1.0 - kVelCmdFilterAlpha) * vel_cmd_filtered_ + kVelCmdFilterAlpha * ref_vel_;
  ref_vel_ = vel_cmd_filtered_;

  // --- 内环: 速度PI → 力矩 ---
  vel_error_ = ref_vel_ - vel_fdb;
  double vel_p = vel_gains_.kp * vel_error_;

  const double vel_integral_threshold = 0.5;  // 0.5 rad/s
  if (std::abs(vel_error_) < vel_integral_threshold) {
    vel_integral_ += vel_error_ * dt;
    vel_integral_ = clamp(vel_integral_, -max_integral_vel_, max_integral_vel_);
  }
  double vel_i = vel_gains_.ki * vel_integral_;

  double vel_d = 0.0;
  if (dt > 1e-6) {
    vel_d = -vel_gains_.kd * (vel_fdb - vel_fdb_prev_) / dt;
  }

  vel_error_prev_ = vel_error_;
  vel_fdb_prev_ = vel_fdb;
  return vel_p + vel_i + vel_d;
}

void CascadePid::reset() {
  pos_error_ = 0.0;
  pos_error_prev_ = 0.0;
  pos_integral_ = 0.0;

  vel_error_ = 0.0;
  vel_error_prev_ = 0.0;
  vel_integral_ = 0.0;

  ref_vel_ = 0.0;
  vel_cmd_filtered_ = 0.0;
  pos_fdb_prev_ = 0.0;
  vel_fdb_prev_ = 0.0;
}

void CascadePid::setPositionGains(const PidGains &gains) {
  pos_gains_ = gains;
}

void CascadePid::setVelocityGains(const PidGains &gains) {
  vel_gains_ = gains;
}

void CascadePid::setMaxVelocity(double max_vel) {
  max_vel_ = max_vel;
}

MultiJointCascadePid::MultiJointCascadePid(size_t num_joints) {
  PidGains default_gains(0.0, 0.0, 0.0);
  for (size_t i = 0; i < num_joints; ++i) {
    controllers_.emplace_back(default_gains, default_gains, 1.0);
  }
}

void MultiJointCascadePid::setJointParams(size_t joint_idx, const PidGains &pos_gains,
                                          const PidGains &vel_gains, double max_vel,
                                          double max_integral_pos, double max_integral_vel) {
  if (joint_idx >= controllers_.size()) return;
  controllers_[joint_idx] =
      CascadePid(pos_gains, vel_gains, max_vel, max_integral_pos, max_integral_vel);
}

void MultiJointCascadePid::compute(const std::vector<double> &pos_ref,
                                   const std::vector<double> &pos_fdb,
                                   const std::vector<double> &vel_fdb, double dt,
                                   std::vector<double> &torque_out) {
  torque_out.resize(controllers_.size());
  for (size_t i = 0; i < controllers_.size(); ++i) {
    torque_out[i] = controllers_[i].compute(pos_ref[i], pos_fdb[i], vel_fdb[i], dt);
  }
}

void MultiJointCascadePid::resetAll() {
  for (auto &controller : controllers_) {
    controller.reset();
  }
}

CascadePid &MultiJointCascadePid::getJointController(size_t joint_idx) {
  return controllers_[joint_idx];
}
