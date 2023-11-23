#include "auto_marker_docking/PD_controller.hpp"

PDController::PDController(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd)
    : kp_(kp), kd_(kd), target_(Eigen::Vector3d::Zero()), previous_error_(Eigen::Vector3d::Zero()) {}

// 목표값 설정 구현
void PDController::set_target(const Eigen::Vector3d& target) {
    target_ = target;
}

// 현재값 업데이트 및 제어 신호 계산 구현
Eigen::Vector3d PDController::update(const Eigen::Vector3d& measurement, double dt) {
    Eigen::Vector3d error = target_ - measurement;
    Eigen::Vector3d derivative = (error - previous_error_) / dt;
    previous_error_ = error;
    return kp_.cwiseProduct(error) + kd_.cwiseProduct(derivative);
}

// 비례 게인 설정 구현
void PDController::setKp(const Eigen::Vector3d& kp) {
    kp_ = kp;
}

// 미분 게인 설정 구현
void PDController::setKd(const Eigen::Vector3d& kd) {
    kd_ = kd;
}

// 비례 게인 가져오기 구현
Eigen::Vector3d PDController::getKp() const {
    return kp_;
}

// 미분 게인 가져오기 구현
Eigen::Vector3d PDController::getKd() const {
    return kd_;
}