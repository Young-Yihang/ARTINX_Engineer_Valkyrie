#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter1D {
private:
  // 状态向量 [位置, 速度]
  Eigen::Vector2d x_;  // 状态估计
  Eigen::Matrix2d P_;  // 误差协方差

  // 系统矩阵
  Eigen::Matrix2d F_;  // 状态转移矩阵
  Eigen::Matrix2d H_;  // 测量矩阵
  Eigen::Matrix2d Q_;  // 过程噪声协方差
  Eigen::Matrix2d R_;  // 测量噪声协方差

  double dt_;  // 时间步长

public:
  KalmanFilter1D(double dt);
  // 初始化状态
  void initialize(double q_init, double qd_init);

  // 预测步骤
  void predict();

  // 更新步骤
  void update(double q_measured, double qd_measured);

  // 获取估计值
  double getPosition() const;
  double getVelocity() const;
  // 获取协方差（用于调试）
  Eigen::Matrix2d getCovariance() const;
  Eigen::Matrix2d getKalmanGain() const;

  // 设置噪声参数（用于在线调参）
  void setProcessNoise(double q_pos, double q_vel);

  void setMeasurementNoise(double r_pos, double r_vel);
};

#endif  // KALMAN_FILTER_HPP