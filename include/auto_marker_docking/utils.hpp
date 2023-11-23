#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

inline constexpr double Half_PI = 3.14159265358979323846 * 0.5;
inline constexpr double PI = 3.14159265358979323846;

inline Eigen::Quaterniond rodrigues_to_quaternion(double rx, double ry, double rz) {
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
    while (radian > PI) {
        radian -= 2 * PI;
    }
    while (radian < -PI) {
        radian += 2 * PI;
    }

    return radian;
}

inline Eigen::Vector2d xy_to_polar_coordinates(double x, double y) {
    double length = std::sqrt(x * x + y * y);
    double angle = radian_normalization(std::atan2(y, x));

    return Eigen::Vector2d(length, angle);
}



#endif