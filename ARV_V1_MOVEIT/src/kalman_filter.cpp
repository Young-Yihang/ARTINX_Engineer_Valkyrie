#include <kalman_filter.hpp>

KalmanFilter1D::KalmanFilter1D(double dt) : dt_(dt)
{
    // 初始化状态
    x_ = Eigen::Vector2d::Zero();

    // 初始化协方差（表示初始不确定性）
    P_ = Eigen::Matrix2d::Identity() * 0.1;

    // 状态转移矩阵
    F_ << 1, dt_,
        0, 1;

    // 测量矩阵
    H_ = Eigen::Matrix2d::Identity();

    // 过程噪声（需要调参）
    Q_ << 1e-5, 0,
        0, 1e-3;

    // 测量噪声（需要调参）
    R_ << 1e-6, 0,
        0, 1e-2;
}

// 初始化状态
void KalmanFilter1D::initialize(double q_init, double qd_init)
{
    x_ << q_init, qd_init;
}

// 预测步骤
void KalmanFilter1D::predict()
{
    // 1. 状态预测
    x_ = F_ * x_;

    // 2. 协方差预测
    P_ = F_ * P_ * F_.transpose() + Q_;
}

// 更新步骤
void KalmanFilter1D::update(double q_measured, double qd_measured)
{
    // 3. 测量向量
    Eigen::Vector2d z;
    z << q_measured, qd_measured;

    // 4. 创新
    Eigen::Vector2d y = z - H_ * x_;

    // 5. 创新协方差
    Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R_;

    // 6. 卡尔曼增益
    Eigen::Matrix2d K = P_ * H_.transpose() * S.inverse();

    // 7. 状态更新
    x_ = x_ + K * y;

    // 8. 协方差更新
    Eigen::Matrix2d I = Eigen::Matrix2d::Identity();
    P_ = (I - K * H_) * P_;
}

double KalmanFilter1D::getPosition() const { return x_(0); }

double KalmanFilter1D::getVelocity() const { return x_(1); }

Eigen::Matrix2d KalmanFilter1D::getCovariance() const { return P_; }

// 设置噪声参数（用于在线调参）
void KalmanFilter1D::setProcessNoise(double q_pos, double q_vel)
{
    Q_(0, 0) = q_pos;
    Q_(1, 1) = q_vel;
}

void KalmanFilter1D::setMeasurementNoise(double r_pos, double r_vel)
{
    R_(0, 0) = r_pos;
    R_(1, 1) = r_vel;
}
