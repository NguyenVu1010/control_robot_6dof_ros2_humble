#include "my_robot_controllers/algo/fk_solver.hpp"
#include <cstdio>

namespace my_robot_controllers {
namespace algo {

FkSolver::FkSolver(const KDL::Tree& tree, 
                   const std::string& base_link, 
                   const std::string& tip_link) 
{
    // Tự tạo Chain nội bộ để tránh lỗi bộ nhớ
    if (!tree.getChain(base_link, tip_link, chain_)) {
        printf("[FkSolver] ERROR: Failed to create chain from %s to %s\n", 
               base_link.c_str(), tip_link.c_str());
        num_joints_ = 0;
        return;
    }

    num_joints_ = chain_.getNrOfJoints();
    solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    
    printf("[FkSolver] Init OK. Joints: %d\n", num_joints_);
}

bool FkSolver::calculate(const KDL::JntArray& q_in, KDL::Frame& p_out) {
    if (num_joints_ == 0) return false;

    // Kiểm tra kích thước đầu vào
    if (q_in.rows() != num_joints_) {
        // printf("[FkSolver] Input mismatch! Q: %d, Chain: %d\n", q_in.rows(), num_joints_);
        return false;
    }

    return (solver_->JntToCart(q_in, p_out) >= 0);
}

} }