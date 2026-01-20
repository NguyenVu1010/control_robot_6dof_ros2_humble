#ifndef MY_ROBOT_CONTROLLERS__ALGO__TYPES_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__TYPES_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

namespace my_robot_controllers {
namespace algo {

// Ma trận Jacobian: 6 hàng (XYZ + RPY), số cột động (tương ứng số khớp)
using Matrix6Xd = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Ma trận vuông 6x6 (Dùng cho tính toán không gian Cartesian)
using Matrix6d = Eigen::Matrix<double, 6, 6>;

// Vector vận tốc khớp (số hàng động)
using VectorXd = Eigen::VectorXd;

// Vector vận tốc Cartesian (6x1): [vx, vy, vz, wx, wy, wz]
using Vector6d = Eigen::Matrix<double, 6, 1>;

} // namespace algo
} // namespace my_robot_controllers

#endif