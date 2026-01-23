#ifndef SKL_TYPES_HPP_
#define SKL_TYPES_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace srk { // srk = Simple Robot Kinematics

// Vector khớp (6x1)
using JntArray = Eigen::VectorXd;

// Ma trận biến đổi (4x4)
using Frame = Eigen::Isometry3d;

// Jacobian (6xn)
using Jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>;

// Vector 6D (v, w)
using Vector6d = Eigen::Matrix<double, 6, 1>;

}
#endif