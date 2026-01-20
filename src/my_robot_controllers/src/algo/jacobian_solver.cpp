#include "my_robot_controllers/algo/jacobian_solver.hpp"
#include <cstdio>

namespace my_robot_controllers {
namespace algo {

JacobianSolver::JacobianSolver(const KDL::Tree& tree, 
                               const std::string& base_link, 
                               const std::string& tip_link)
{
    // 1. Tự tạo Chain tại chỗ (Local Chain Generation)
    // Điều này ngăn chặn lỗi bộ nhớ khi truyền object qua lại giữa các file
    if (!tree.getChain(base_link, tip_link, chain_)) {
        printf("[JacobianSolver] FATAL: Failed to create chain internally!\n");
        num_joints_ = 0;
        return;
    }

    num_joints_ = chain_.getNrOfJoints();
    printf("[JacobianSolver] Internal Chain Created. Joints: %d\n", num_joints_);

    // 2. Init Solver
    solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    kdl_jacobian_.resize(num_joints_);
}

bool JacobianSolver::calculate(const KDL::JntArray& q_in, Matrix6Xd& jacobian_out) {
    if (num_joints_ == 0) return false;

    // 1. Check Input
    if (q_in.rows() != num_joints_) {
        printf("[JacSolver] Input Size Mismatch (%d vs %d)\n", q_in.rows(), num_joints_);
        return false;
    }

    // 2. Check Buffer
    if (kdl_jacobian_.columns() != num_joints_) {
        kdl_jacobian_.resize(num_joints_);
    }

    // 3. Solve
    int ret = solver_->JntToJac(q_in, kdl_jacobian_);
    if (ret < 0) {
        printf("[JacSolver] KDL Error: %d\n", ret);
        return false;
    }

    jacobian_out = kdl_jacobian_.data;
    return true;
}

} }