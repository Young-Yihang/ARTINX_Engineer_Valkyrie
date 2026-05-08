/// @file cascade_pid.hpp
/// @brief Cascade PID: outer pos→vel, inner vel→torque
#ifndef CASCADE_PID_HPP
#define CASCADE_PID_HPP

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

struct PidGains {
  double kp, ki, kd;
  PidGains() : kp(0.0), ki(0.0), kd(0.0) {}
  PidGains(double p, double i, double d) : kp(p), ki(i), kd(d) {}
};

class CascadePid {
public:
  CascadePid(const PidGains &pos_gains, const PidGains &vel_gains, double max_vel,
             double max_integral_pos = 1e10, double max_integral_vel = 1e10);

  /// @return torque (N·m)
  double compute(double pos_ref, double pos_fdb, double vel_fdb, double dt);
  void reset();

  void setPositionGains(const PidGains &gains);
  void setVelocityGains(const PidGains &gains);
  void setMaxVelocity(double max_vel);

  double getRefVelocity() const { return ref_vel_; }
  double getPositionError() const { return pos_error_; }
  double getVelocityError() const { return vel_error_; }

  void setContinuous(bool c) { is_continuous_ = c; }
  void setDerivativeMode(uint32_t sample_period_ms, bool divide_dt);

private:
  // --- 位置环(外环) ---
  PidGains pos_gains_;
  double pos_error_;
  double pos_error_prev_;
  double pos_fdb_prev_;  // 反馈微分用
  double pos_integral_;
  double max_integral_pos_;

  // --- 速度环(内环) ---
  PidGains vel_gains_;
  double vel_error_;
  double vel_error_prev_;
  double vel_fdb_prev_;  // 速度环反馈微分用
  double vel_integral_;
  double max_integral_vel_;

  // --- D项低采样 (对齐MCU Pid::SetDerivativeMode) ---
  uint32_t d_sample_period_ms_ = 1;
  bool d_divide_dt_ = true;
  double d_dt_inv_ = 1000.0;  // 1.0 / (d_sample_period_ms_ * 0.001), precomputed
  uint32_t d_sample_counter_ = 0;
  double pos_fdb_lowsample_[2] = {};
  double pos_d_held_ = 0.0;

  // --- loop处理
  double max_vel_;  // rad/s
  double ref_vel_;  // 外环输出参考速度 (rad/s)
  bool is_continuous_ = false;

  inline double angleDiff(double a, double b) const {
    if (!std::isfinite(a) || !std::isfinite(b)) return 0.0;
    return is_continuous_ ? std::remainder(a - b, 2.0 * M_PI) : (a - b);
  }
};

/// Multi-joint cascade PID manager
class MultiJointCascadePid {
public:
  MultiJointCascadePid(size_t num_joints);

  void setJointParams(size_t joint_idx, const PidGains &pos_gains, const PidGains &vel_gains,
                      double max_vel, double max_integral_pos = 1e10,
                      double max_integral_vel = 1e10);

  void compute(const std::vector<double> &pos_ref, const std::vector<double> &pos_fdb,
               const std::vector<double> &vel_fdb, double dt, std::vector<double> &torque_out);

  void resetAll();

  CascadePid &getJointController(size_t joint_idx);

  void setJointContinuous(size_t joint_idx, bool c) {
    if (joint_idx < controllers_.size()) controllers_[joint_idx].setContinuous(c);
  }

private:
  std::vector<CascadePid> controllers_;
};

#endif  // CASCADE_PID_HPP
