#ifndef SKL_KINEMATICS_CORE_HPP_
#define SKL_KINEMATICS_CORE_HPP_

#include "chain.hpp"
#include <memory>

namespace srk {

class KinematicsCore {
public:
    KinematicsCore();
    
    // Khởi tạo từ chuỗi URDF XML
    bool init(const std::string& urdf_xml, 
              const std::string& base_link, 
              const std::string& tip_link);

    // Tính FK (Vị trí điểm cuối)
    // Input: q (góc khớp) -> Output: pose (XYZ RPY)
    bool solveFK(const JntArray& q, Frame& out_pose);

    // Tính IK vận tốc (Differential IK)
    // Input: q_current, v_cartesian -> Output: q_dot
    bool solveIK_Velocity(const JntArray& q, const Vector6d& v_cart, JntArray& q_dot_out);

    // Tính toán vận tốc khớp cần thiết để đến đích (P-Controller tích hợp)
    // Input: q_current, target_pose, dt -> Output: q_dot
    bool solveIK_Position(const JntArray& q, const Frame& target_pose, JntArray& q_dot_out);

    unsigned int getNrOfJoints() const;

private:
    Chain chain_;
    unsigned int n_joints_;
    bool initialized_ = false;
    void internal_compute_jacobian(const JntArray& q, Jacobian& J);
};

}
#endif