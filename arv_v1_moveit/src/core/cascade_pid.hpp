#ifndef CASCADE_PID_HPP
#define CASCADE_PID_HPP

#include <algorithm>
#include <cmath>
#include <vector>

/**
 * @brief 单个PID控制器的增益参数
 */
struct PidGains {
  double kp;  // 比例增益
  double ki;  // 积分增益
  double kd;  // 微分增益

  PidGains() : kp(0.0), ki(0.0), kd(0.0) {}
  PidGains(double p, double i, double d) : kp(p), ki(i), kd(d) {}
};

/**
 * @brief 单关节级联PID: 外环P(位置→速度), 内环PI(速度→力矩)
 *
 *   - 速度饱和保护
 *   - 微分滤波 (可选,降低噪声影响)
 */
class CascadePid {
public:
  /**
   * @brief 构造函数
   * @param pos_gains 位置环PID增益
   * @param vel_gains 速度环PID增益
   * @param max_vel 速度饱和限制 (rad/s)
   * @param max_integral_pos 位置环积分限幅
   * @param max_integral_vel 速度环积分限幅
   */
  CascadePid(const PidGains &pos_gains, const PidGains &vel_gains, double max_vel,
             double max_integral_pos = 1e10, double max_integral_vel = 1e10);

  /**
   * @brief 计算级联PID控制输出
   * @param pos_ref 参考位置 (rad)
   * @param pos_fdb 反馈位置 (rad)
   * @param vel_fdb 反馈速度 (rad/s)
   * @param dt 控制周期 (s)
   * @return 控制力矩 (N·m)
   */
  double compute(double pos_ref, double pos_fdb, double vel_fdb, double dt);

  /**
   * @brief 重置积分器和历史误差
   */
  void reset();

  /**
   * @brief 设置位置环增益
   */
  void setPositionGains(const PidGains &gains);

  /**
   * @brief 设置速度环增益
   */
  void setVelocityGains(const PidGains &gains);

  /**
   * @brief 设置速度饱和限制
   */
  void setMaxVelocity(double max_vel);

  /**
   * @brief 获取当前参考速度 (用于调试)
   */
  double getRefVelocity() const { return ref_vel_; }

  /**
   * @brief 获取位置误差 (用于调试)
   */
  double getPositionError() const { return pos_error_; }

  /**
   * @brief 获取速度误差 (用于调试)
   */
  double getVelocityError() const { return vel_error_; }

private:
  // --- 位置环(外环)参数 ---
  PidGains pos_gains_;       // 位置环PID增益
  double pos_error_;         // 位置误差 e_p = ref - fdb
  double pos_error_prev_;    // 上一次位置误差 (用于微分)
  double pos_integral_;      // 位置误差积分
  double max_integral_pos_;  // 位置积分限幅

  // --- 速度环(内环)参数 ---
  PidGains vel_gains_;       // 速度环PID增益
  double vel_error_;         // 速度误差 e_v = ref - fdb
  double vel_error_prev_;    // 上一次速度误差 (用于微分)
  double vel_integral_;      // 速度误差积分
  double max_integral_vel_;  // 速度积分限幅

  // --- 饱和限制 ---
  double max_vel_;  // 速度饱和限制 (rad/s)

  // --- 内部状态 ---
  double ref_vel_;  // 外环输出的参考速度 (rad/s)

  /**
   * @brief 限幅函数
   */
  inline double clamp(double value, double min_val, double max_val) const {
    return std::max(min_val, std::min(value, max_val));
  }
};

/**
 * @brief 多关节的级联PID控制器
 *
 * 管理6个关节的级联PID控制器
 */
class MultiJointCascadePid {
public:
  /**
   * @brief 构造函数
   * @param num_joints 关节数量 (通常为6)
   */
  MultiJointCascadePid(size_t num_joints);

  /**
   * @brief 为指定关节设置PID参数
   * @param joint_idx 关节索引 (0-5)
   * @param pos_gains 位置环增益
   * @param vel_gains 速度环增益
   * @param max_vel 速度限制
   */
  void setJointParams(size_t joint_idx, const PidGains &pos_gains, const PidGains &vel_gains,
                      double max_vel, double max_integral_pos = 1e10,
                      double max_integral_vel = 1e10);

  /**
   * @brief 计算所有关节的控制力矩
   * @param pos_ref 参考位置向量 (6维)
   * @param pos_fdb 反馈位置向量 (6维)
   * @param vel_fdb 反馈速度向量 (6维)
   * @param dt 控制周期 (s)
   * @param torque_out 输出力矩向量 (6维)
   */
  void compute(const std::vector<double> &pos_ref, const std::vector<double> &pos_fdb,
               const std::vector<double> &vel_fdb, double dt, std::vector<double> &torque_out);

  /**
   * @brief 重置所有关节的积分器
   */
  void resetAll();

  /**
   * @brief 获取单个关节的控制器 (用于调试)
   */
  CascadePid &getJointController(size_t joint_idx);

private:
  std::vector<CascadePid> controllers_;  // 每个关节一个级联PID
};

#endif  // CASCADE_PID_HPP
