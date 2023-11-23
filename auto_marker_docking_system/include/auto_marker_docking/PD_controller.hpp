// PDController.hpp

#ifndef PDCONTROLLER_HPP
#define PDCONTROLLER_HPP

#include <Eigen/Dense>

class PDController {
public:
    // 생성자
    PDController(const Eigen::Vector3d& kp, const Eigen::Vector3d& kd);

    // 목표값 설정
    void set_target(const Eigen::Vector3d& target);

    // 현재값 업데이트 및 제어 신호 계산
    Eigen::Vector3d update(const Eigen::Vector3d& measurement, double dt);

    // 게인 설정
    void setKp(const Eigen::Vector3d& kp);
    void setKd(const Eigen::Vector3d& kd);

    // 게인 가져오기
    Eigen::Vector3d getKp() const;
    Eigen::Vector3d getKd() const;

private:
    Eigen::Vector3d kp_; // 비례 게인
    Eigen::Vector3d kd_; // 미분 게인
    Eigen::Vector3d target_; // 목표값
    Eigen::Vector3d previous_error_; // 이전 스텝의 오차
};

#endif // PDCONTROLLER_HPP
