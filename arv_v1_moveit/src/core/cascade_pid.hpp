/// @file cascade_pid.hpp
/// @brief Cascade PID: outer pos→vel, inner vel→torque
#ifndef CASCADE_PID_HPP
#define CASCADE_PID_HPP

#include <algorithm>
#include <cmath>
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

private:
  // --- 位置环(外环) ---
  PidGains pos_gains_;
  double pos_error_;
  double pos_error_prev_;
  double pos_fdb_prev_;       // 反馈微分用
  double pos_integral_;
  double max_integral_pos_;

  // --- 速度环(内环) ---
  PidGains vel_gains_;
  double vel_error_;
  double vel_error_prev_;
  double vel_fdb_prev_;       // 速度环反馈微分用
  double vel_integral_;
  double max_integral_vel_;

  // --- Extra Stage特殊滤波处理 + loop处理
  double max_vel_;  // rad/s
  double ref_vel_;  // 外环输出参考速度 (rad/s)
  double vel_cmd_filtered_;   // vel_cmd 一阶滤波状态
  bool is_continuous_ = false;
  static constexpr double kVelCmdFilterAlpha = 0.1; // a = dt/tau, tau=10ms, dt=1ms

  inline double clamp(double value, double min_val, double max_val) const {
    return std::max(min_val, std::min(value, max_val));
  }

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
