
#ifndef ARUCO_EXTEND_KALMAN_FILTER_HPP
#define ARUCO_EXTEND_KALMAN_FILTER_HPP

#include <Eigen/Dense>

class ArucoExtendKalmanFilter {
public:
    ArucoExtendKalmanFilter(const Eigen::Vector3d& X, const Eigen::Matrix2d& P, const Eigen::Matrix2d& Q, const Eigen::Matrix2d& R)
        : X_(X), P_(P), Q_(Q), R_(R) {};

    void predict(const Eigen::Vector2d& u);
    void update(const Eigen::Vector3d& z);

    void set(const Eigen::Vector3d& X) { X_ = X; };

    Eigen::Vector3d getState() { return X_; }

private:
    Eigen::Vector3d X_; // state vector
    Eigen::Matrix3d F_; // predict space transform matrix 
    Eigen::Matrix3d P_; // state coverience matrix
    Eigen::Matrix3d J_pred_; // predict jacobian
    Eigen::Matrix3d Q_; // predict step noise matrix
    Eigen::Matrix3d R_; // measure step noise matrix
    Eigen::Matrix3d H_; // measure space transform matrix
    Eigen::Matrix3d J_mea_; // measure jacobian
    Eigen::Matrix<double, 2, 3> B_; // input space transform matrix
};

#endif // ARUCO_EXTEND_KALMAN_FILTER_HPP
