#include "my_robot_controllers/algo/fk_solver.hpp"

namespace my_robot_controllers {
namespace algo {

FkSolver::FkSolver(const KDL::Chain& chain) {
    // KDL ChainFkSolverPos_recursive rất nhẹ và an toàn cho RT sau khi init
    solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain);
}

bool FkSolver::calculate(const KDL::JntArray& q_in, KDL::Frame& p_out) {
    return (solver_->JntToCart(q_in, p_out) >= 0);
}

} // namespace algo
} // namespace my_robot_controllers