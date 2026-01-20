#ifndef MY_ROBOT_CONTROLLERS__ALGO__FK_SOLVER_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__FK_SOLVER_HPP_

#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <memory>
#include <string>

namespace my_robot_controllers {
namespace algo {

class FkSolver {
public:
    // Sửa Constructor: Nhận Tree thay vì Chain
    FkSolver(const KDL::Tree& tree, 
             const std::string& base_link, 
             const std::string& tip_link);
    
    bool calculate(const KDL::JntArray& q_in, KDL::Frame& p_out);

private:
    KDL::Chain chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_;
    unsigned int num_joints_;
};

} }
#endif