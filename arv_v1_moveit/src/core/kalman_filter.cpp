/// @file kalman_filter.cpp
/// @brief 1D Kalman filter: x=[pos,vel], predict/update cycle.
#include <kalman_filter.hpp>

KalmanFilter1D::KalmanFilter1D(double dt) : dt_(dt) {
  x_ = Eigen::Vector2d::Zero();
  P_ = Eigen::Matrix2d::Identity() * 0.1;
  F_ << 1, dt_, 0, 1;
  H_ = Eigen::Matrix2d::Identity();
  Q_ << 1e-5, 0, 0, 1e-3;  // 过程噪声 (需调参)
  R_ << 1e-6, 0, 0, 1e-2;  // 测量噪声 (需调参)
}

void KalmanFilter1D::initialize(double q_init, double qd_init) {
  x_ << q_init, qd_init;
}

void KalmanFilter1D::predict() {
  x_ = F_ * x_;
  P_ = F_ * P_ * F_.transpose() + Q_;
}

void KalmanFilter1D::update(double q_measured, double qd_measured) {
  Eigen::Vector2d z(q_measured, qd_measured);
  Eigen::Vector2d y = z - H_ * x_;  // innovation
  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * y;
  P_ = (Eigen::Matrix2d::Identity() - K * H_) * P_;
}

double KalmanFilter1D::getPosition() const {
  return x_(0);
}

double KalmanFilter1D::getVelocity() const {
  return x_(1);
}

Eigen::Matrix2d KalmanFilter1D::getCovariance() const {
  return P_;
}

Eigen::Matrix2d KalmanFilter1D::getKalmanGain() const {
  Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;
  Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse();
  return K;
}

void KalmanFilter1D::setProcessNoise(double q_pos, double q_vel) {
  Q_(0, 0) = q_pos;
  Q_(1, 1) = q_vel;
}

void KalmanFilter1D::setMeasurementNoise(double r_pos, double r_vel) {
  R_(0, 0) = r_pos;
  R_(1, 1) = r_vel;
}
