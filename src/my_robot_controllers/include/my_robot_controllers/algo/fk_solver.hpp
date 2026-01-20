#ifndef MY_ROBOT_CONTROLLERS__ALGO__FK_SOLVER_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__FK_SOLVER_HPP_

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <memory>

namespace my_robot_controllers {
namespace algo {

class FkSolver {
public:
    explicit FkSolver(const KDL::Chain& chain);
    
    // Tính toán vị trí/góc xoay (Frame) của End-Effector từ góc khớp
    // Dùng để debug hoặc kiểm tra giới hạn
    bool calculate(const KDL::JntArray& q_in, KDL::Frame& p_out);

private:
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> solver_;
};

} // namespace algo
} // namespace my_robot_controllers

#endif