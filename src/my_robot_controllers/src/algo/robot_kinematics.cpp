#include "my_robot_controllers/algo/robot_kinematics.hpp"

namespace my_robot_controllers {
namespace algo {

RobotKinematics::RobotKinematics(const KDL::Tree& tree, 
                                 const std::string& base_link, 
                                 const std::string& tip_link) 
{
    // Tạo Solver bằng Tree
    jac_solver_ = std::make_unique<JacobianSolver>(tree, base_link, tip_link);
    
    // Tự tạo chain tạm để đếm khớp phục vụ Math Solver
    KDL::Chain dummy_chain;
    tree.getChain(base_link, tip_link, dummy_chain);
    num_joints_ = dummy_chain.getNrOfJoints();

    math_solver_ = std::make_unique<DlsMath>(num_joints_);
    jacobian_mat_.resize(6, num_joints_);
    jacobian_mat_.setZero();
}

bool RobotKinematics::convertCartToJnt(const KDL::JntArray& q_current, 
                                       const Vector6d& v_target, 
                                       VectorXd& q_dot_cmd) 
{
    if (!jac_solver_->calculate(q_current, jacobian_mat_)) {
        return false;
    }
    math_solver_->solve(jacobian_mat_, v_target, q_dot_cmd);
    return true;
}

} }