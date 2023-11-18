#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

inline constexpr double Half_PI = 3.14159265358979323846 * 0.5;

inline Eigen::Quaterniond rodrigues_to_quaternion(double rx, double ry, double rz) {
    // 벡터의 크기를 계산하여 각도(θ)를 얻습니다.
    double theta = std::sqrt(rx * rx + ry * ry + rz * rz);

    // 단위 벡터를 계산합니다.
    double ux = rx / theta;
    double uy = ry / theta;
    double uz = rz / theta;

    // 쿼터니언 계산
    double halfTheta = theta / 2.0;
    double sinHalfTheta = std::sin(halfTheta);

    Eigen::Quaterniond q(std::cos(halfTheta), ux * sinHalfTheta, uy * sinHalfTheta, uz * sinHalfTheta);

    return q;
}

inline double radian_normalization(double radian)
{
    if (radian < Half_PI * -1) {
        radian = Half_PI + (radian + Half_PI);
    } else if (radian > Half_PI) {
        radian = (Half_PI * -1) + (radian - Half_PI);
    }

    return radian;
}

#endif