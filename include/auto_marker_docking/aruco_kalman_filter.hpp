
#ifndef ARUCO_EXTEND_KALMAN_FILTER_HPP
#define ARUCO_EXTEND_KALMAN_FILTER_HPP

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "auto_marker_docking/utils.hpp"

class ArucoKalmanFilter {
public:
    ArucoKalmanFilter(const Eigen::Vector3d& X, const Eigen::Matrix3d& P, const Eigen::Matrix3d& Q, const Eigen::Matrix3d& R)
        : X_(X), P_(P), Q_(Q), R_(R) {
            initialize();
        };

    void initialize();

    void predict(const Eigen::Vector3d& u);
    void update(const Eigen::Vector3d& z);

    void set(const Eigen::Vector3d& X) { X_ = X; };

    Eigen::Isometry3d get_state() {
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        Eigen::AngleAxisd rotation(X_(2), Eigen::Vector3d::UnitZ());

        pose.translate(Eigen::Vector3d(X_(0), X_(1), 0));
        pose.rotate(rotation);

        return pose;
    }
private:
    Eigen::Vector3d X_; // state vector
    Eigen::Matrix3d F_; // predict space transform matrix 
    Eigen::Matrix3d P_; // state coverience matrix
    Eigen::Matrix3d Q_; // predict step noise matrix
    Eigen::Matrix3d R_; // measure step noise matrix
    Eigen::Matrix3d H_; // measure space transform matrix
    Eigen::Matrix3d B_; // input space transform matrix
};

#endif // ARUCO_EXTEND_KALMAN_FILTER_HPP
