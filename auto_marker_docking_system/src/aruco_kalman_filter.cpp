#include "auto_marker_docking/aruco_kalman_filter.hpp"

void ArucoKalmanFilter::initialize()
{
    F_ = Eigen::Matrix3d::Identity();
    B_ = Eigen::Matrix3d::Identity() * -1.0;
    H_ = Eigen::Matrix3d::Identity();
}

void ArucoKalmanFilter::predict(const Eigen::Vector3d& u)
{
    X_ = F_ * X_ + B_ * u;
    P_ = F_ * P_ * F_.transpose() + Q_;
    X_(2) = radian_normalization(X_(2));
}

void ArucoKalmanFilter::update(const Eigen::Vector3d& z)
{
    Eigen::Vector3d y = z - H_ * X_;
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;
    Eigen::Matrix<double, 3, 3> K = P_ * H_.transpose() * S.inverse();

    y(2) = radian_normalization(y(2));

    X_ = X_ + K * y;
    X_(2) = radian_normalization(X_(2));

    P_ = (Eigen::Matrix3d::Identity() - K * H_) * P_;
}

