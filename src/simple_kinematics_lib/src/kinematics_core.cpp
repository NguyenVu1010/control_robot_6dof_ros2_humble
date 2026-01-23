#include "simple_kinematics_lib/kinematics_core.hpp"
#include <iostream>
#include <cmath>

// Khai báo hàm parse ở trên (forward declaration)
namespace srk {
    bool parseURDF(const std::string& xml, const std::string& base, const std::string& tip, Chain& chain);
}

namespace srk {

KinematicsCore::KinematicsCore() : n_joints_(0) {}

bool KinematicsCore::init(const std::string& urdf_xml, const std::string& base, const std::string& tip) {
    chain_ = Chain(); // Reset
    if (!parseURDF(urdf_xml, base, tip, chain_)) {
        std::cerr << "[KinematicsCore] Failed to parse URDF!" << std::endl;
        return false;
    }
    n_joints_ = chain_.getNrOfJoints();
    initialized_ = true;
    std::cout << "[KinematicsCore] Init success. Joints: " << n_joints_ << std::endl;
    return true;
}

unsigned int KinematicsCore::getNrOfJoints() const { return n_joints_; }

// --- FORWARD KINEMATICS ---
bool KinematicsCore::solveFK(const JntArray& q, Frame& out_pose) {
    if (!initialized_ || q.size() != n_joints_) return false;

    out_pose = Frame::Identity();
    int q_idx = 0;
    for (const auto& seg : chain_.getSegments()) {
        double val = (seg.type != FIXED) ? q(q_idx++) : 0.0;
        out_pose = out_pose * seg.pose(val);
    }
    return true;
}

// --- JACOBIAN & IK ---
bool KinematicsCore::solveIK_Velocity(const JntArray& q, const Vector6d& v_cart, JntArray& q_dot_out) {
    if (!initialized_ || q.size() != n_joints_) return false;

    // 1. Tính Jacobian (Geometric)
    Jacobian J(6, n_joints_);
    J.setZero();
    
    Frame T_curr = Frame::Identity();
    std::vector<Eigen::Vector3d> z_axes;
    std::vector<Eigen::Vector3d> p_origins;
    std::vector<int> q_map;

    int q_idx = 0;
    for (const auto& seg : chain_.getSegments()) {
        if (seg.type != FIXED) {
            z_axes.push_back(T_curr.linear() * seg.axis);
            p_origins.push_back(T_curr.translation());
            q_map.push_back(q_idx);
            T_curr = T_curr * seg.pose(q(q_idx++));
        } else {
            T_curr = T_curr * seg.pose(0.0);
        }
    }
    Eigen::Vector3d p_ee = T_curr.translation();

    for (unsigned int i = 0; i < n_joints_; ++i) {
        Eigen::Vector3d z = z_axes[i];
        Eigen::Vector3d p = p_origins[i];
        J.block<3, 1>(0, i) = z.cross(p_ee - p); // Linear v
        J.block<3, 1>(3, i) = z;                 // Angular w
    }

    // 2. Damped Least Squares: q_dot = J^T * (J*J^T + lambda^2*I)^-1 * v
    double lambda = 0.01; // Damping factor
    Eigen::MatrixXd JJT = J * J.transpose();
    JJT.diagonal().array() += lambda * lambda;
    
    // Solve
    q_dot_out = J.transpose() * JJT.ldlt().solve(v_cart);
    return true;
}

bool KinematicsCore::solveIK_Position(const JntArray& q, const Frame& target, JntArray& q_dot_out) {
    Frame current;
    solveFK(q, current);

    // Tính sai số vị trí (Linear)
    Eigen::Vector3d p_err = target.translation() - current.translation();
    
    // Tính sai số góc (Angular) - Dùng Quaternion
    Eigen::Quaterniond q_cur(current.linear());
    Eigen::Quaterniond q_tar(target.linear());
    // q_diff = q_target * q_current^-1
    Eigen::Quaterniond q_diff = q_tar * q_cur.inverse(); 
    Eigen::AngleAxisd aa(q_diff);
    Eigen::Vector3d w_err = aa.axis() * aa.angle();

    Vector6d v_cmd;
    v_cmd << p_err, w_err;

    // P-Controller Gain
    double Kp_lin = 5.0;
    double Kp_ang = 2.0;
    
    // Clamp limits
    for(int i=0; i<3; ++i) {
        if(v_cmd(i) > 0.5) v_cmd(i) = 0.5;
        if(v_cmd(i) < -0.5) v_cmd(i) = -0.5;
        if(v_cmd(i+3) > 1.0) v_cmd(i+3) = 1.0;
        if(v_cmd(i+3) < -1.0) v_cmd(i+3) = -1.0;
    }
    
    v_cmd.head(3) *= Kp_lin;
    v_cmd.tail(3) *= Kp_ang;

    return solveIK_Velocity(q, v_cmd, q_dot_out);
}

}