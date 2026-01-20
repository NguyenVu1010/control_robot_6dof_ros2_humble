#ifndef MY_ROBOT_CONTROLLERS__ALGO__JACOBIAN_SOLVER_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__JACOBIAN_SOLVER_HPP_

#include "types.hpp"
#include <kdl/tree.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <memory>
#include <string>

namespace my_robot_controllers {
namespace algo {

class JacobianSolver {
public:
    // SỬA: Nhận Tree và tên Link để tự tạo Chain nội bộ
    JacobianSolver(const KDL::Tree& tree, 
                   const std::string& base_link, 
                   const std::string& tip_link);

    bool calculate(const KDL::JntArray& q_in, Matrix6Xd& jacobian_out);

private:
    unsigned int num_joints_;
    KDL::Chain chain_; // Lưu chain nội bộ
    std::unique_ptr<KDL::ChainJntToJacSolver> solver_;
    KDL::Jacobian kdl_jacobian_;
};

} }
#endif