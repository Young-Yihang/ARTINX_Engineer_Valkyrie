/// @file kalman_filter.hpp
/// @brief 1D Kalman filter: x=[pos,vel] state estimation per joint.
#ifndef KALMAN_FILTER_HPP
#define KALMAN_FILTER_HPP

#include <Eigen/Dense>

class KalmanFilter1D {
private:
  Eigen::Vector2d x_;  // state [pos, vel]
  Eigen::Matrix2d P_;  // error covariance
  Eigen::Matrix2d F_;  // state transition
  Eigen::Matrix2d H_;  // measurement
  Eigen::Matrix2d Q_;  // process noise
  Eigen::Matrix2d R_;  // measurement noise
  double dt_;

public:
  KalmanFilter1D(double dt);
  void initialize(double q_init, double qd_init);
  void predict();
  void update(double q_measured, double qd_measured);

  double getPosition() const;
  double getVelocity() const;
  Eigen::Matrix2d getCovariance() const;
  Eigen::Matrix2d getKalmanGain() const;

  void setProcessNoise(double q_pos, double q_vel);
  void setMeasurementNoise(double r_pos, double r_vel);
};

#endif  // KALMAN_FILTER_HPP