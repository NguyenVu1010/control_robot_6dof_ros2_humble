#ifndef MY_ROBOT_CONTROLLERS__ALGO__ROBOT_KINEMATICS_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__ROBOT_KINEMATICS_HPP_

#include "fk_solver.hpp"
#include "jacobian_solver.hpp"
#include "math_utils.hpp"
#include <kdl/tree.hpp>

namespace my_robot_controllers {
namespace algo {

class RobotKinematics {
public:
    // SỬA: Nhận Tree thay vì Chain
    RobotKinematics(const KDL::Tree& tree, 
                    const std::string& base_link, 
                    const std::string& tip_link);

    bool convertCartToJnt(const KDL::JntArray& q_current, 
                          const Vector6d& v_target, 
                          VectorXd& q_dot_cmd);
    
    // Helper để lấy Chain cho Controller resize biến
    unsigned int getNrOfJoints() const { return num_joints_; }

private:
    unsigned int num_joints_;
    std::unique_ptr<JacobianSolver> jac_solver_;
    std::unique_ptr<DlsMath> math_solver_;
    Matrix6Xd jacobian_mat_;
};

} }
#endif