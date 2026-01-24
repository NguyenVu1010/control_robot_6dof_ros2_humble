#include "simple_kinematics_lib/kinematics_core.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>

// Forward declaration của hàm parser
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
        // 1. Đi từ Link Cha đến Khớp (Origin)
        out_pose = out_pose * seg.T_origin;
        
        // 2. Thực hiện biến đổi khớp
        double val = (seg.type != FIXED) ? q(q_idx++) : 0.0;
        out_pose = out_pose * seg.jointTransform(val);
    }
    return true;
}

// --- HÀM NỘI BỘ: TÍNH JACOBIAN ---
// (Dùng chung cho cả Velocity và Position Solver)
void KinematicsCore::internal_compute_jacobian(const JntArray& q, Jacobian& J) {
    if (J.cols() != n_joints_) J.resize(6, n_joints_);
    J.setZero();

    Frame T_curr = Frame::Identity();
    std::vector<Eigen::Vector3d> z_axes;
    std::vector<Eigen::Vector3d> p_joints;

    int q_idx = 0;
    for (const auto& seg : chain_.getSegments()) {
        // Đến vị trí đặt khớp
        Frame T_joint_base = T_curr * seg.T_origin;

        if (seg.type != FIXED) {
            // Lưu trục quay Z và vị trí tâm khớp trong hệ Global
            z_axes.push_back(T_joint_base.linear() * seg.axis);
            p_joints.push_back(T_joint_base.translation());
            
            // Update trạng thái sau khi khớp quay
            T_curr = T_joint_base * seg.jointTransform(q(q_idx++));
        } else {
            T_curr = T_joint_base;
        }
    }

    Eigen::Vector3d p_ee = T_curr.translation();

    // Điền ma trận Jacobian
    for (unsigned int i = 0; i < n_joints_; ++i) {
        Eigen::Vector3d z = z_axes[i];
        Eigen::Vector3d p = p_joints[i];
        
        // Linear v = z x r
        J.block<3, 1>(0, i) = z.cross(p_ee - p); 
        // Angular w = z
        J.block<3, 1>(3, i) = z;                 
    }
}

// --- INVERSE KINEMATICS (VELOCITY LEVEL) ---
// Dùng Adaptive DLS để mượt mà
bool KinematicsCore::solveIK_Velocity(const JntArray& q, const Vector6d& v_cart, JntArray& q_dot_out) {
    if (!initialized_ || q.size() != n_joints_) return false;

    // 1. Tính Jacobian
    Jacobian J(6, n_joints_);
    internal_compute_jacobian(q, J);

    // 2. Adaptive DLS (Damping động)
    // Giúp chính xác khi ở xa điểm kỳ dị, và an toàn khi lại gần
    Eigen::MatrixXd JJT = J * J.transpose();
    double manipulability = std::sqrt(JJT.determinant()); // Độ đo khả năng vận động
    
    double lambda = 0.0;       // Mặc định không damping (Chính xác nhất)
    double w_threshold = 0.02; // Ngưỡng bắt đầu kích hoạt bảo vệ
    double lambda_max = 0.1;   // Damping tối đa

    if (manipulability < w_threshold) {
        double ratio = (1.0 - manipulability / w_threshold);
        lambda = lambda_max * ratio * ratio; 
    }

    // Giải phương trình: (J*J^T + lambda^2*I) * x = v
    JJT.diagonal().array() += lambda * lambda;
    q_dot_out = J.transpose() * JJT.ldlt().solve(v_cart);
    
    // Safety clamp (Giới hạn tốc độ khớp)
    double max_speed = 3.0; 
    for(int i=0; i<n_joints_; ++i) {
        if(q_dot_out(i) > max_speed) q_dot_out(i) = max_speed;
        if(q_dot_out(i) < -max_speed) q_dot_out(i) = -max_speed;
    }

    return true;
}

// --- INVERSE KINEMATICS (POSITION LEVEL) ---
// Dùng phương pháp Newton-Raphson Iterative
bool KinematicsCore::solveIK_Position(const JntArray& q_current, const Frame& target, JntArray& q_dot_out) {
    if (!initialized_) return false;

    // Biến tạm để lặp
    JntArray q_sol = q_current;
    Frame current_pose;
    Jacobian J(6, n_joints_);
    
    // Cấu hình vòng lặp
    int max_iter = 10;       // Số lần lặp tối đa (10 là đủ cho realtime 1ms)
    double eps_pos = 1e-4;   // Sai số vị trí cho phép (0.1mm)
    double eps_rot = 1e-3;   // Sai số góc cho phép

    for (int iter = 0; iter < max_iter; ++iter) {
        // 1. Tính FK tại vị trí khớp ảo q_sol
        solveFK(q_sol, current_pose);

        // 2. Tính sai số (Error Twist)
        Eigen::Vector3d p_err = target.translation() - current_pose.translation();
        
        Eigen::Quaterniond q_cur_R(current_pose.linear());
        Eigen::Quaterniond q_tar_R(target.linear());
        // q_diff = q_target * q_current^-1
        Eigen::Quaterniond q_diff = q_tar_R * q_cur_R.inverse();
        Eigen::AngleAxisd aa(q_diff);
        Eigen::Vector3d w_err = aa.axis() * aa.angle();

        Vector6d err_vec;
        err_vec << p_err, w_err;

        // Check hội tụ
        if (p_err.norm() < eps_pos && w_err.norm() < eps_rot) {
            break; // Đã tìm thấy nghiệm chính xác
        }

        // 3. Tính Jacobian tại q_sol
        internal_compute_jacobian(q_sol, J);

        // 4. Tính delta_q để sửa lỗi (Dùng DLS nhẹ để ổn định vòng lặp)
        double lambda = 0.05; 
        Eigen::MatrixXd JJT = J * J.transpose();
        JJT.diagonal().array() += lambda * lambda;
        
        JntArray delta_q = J.transpose() * JJT.ldlt().solve(err_vec);
        
        // Cập nhật nghiệm
        q_sol += delta_q;
    }

    // SAU KHI TÌM ĐƯỢC GÓC KHỚP ĐÍCH (q_sol)
    // Tính vận tốc điều khiển theo kiểu P-Controller trong không gian khớp
    // v = Kp * (q_target - q_current)
    
    double Kp_joint = 10.0; // Gain phản hồi khớp
    q_dot_out = (q_sol - q_current) * Kp_joint;

    // Clamp output lần cuối
    double max_speed = 2.0;
    for(int i=0; i<n_joints_; ++i) {
        if(q_dot_out(i) > max_speed) q_dot_out(i) = max_speed;
        if(q_dot_out(i) < -max_speed) q_dot_out(i) = -max_speed;
    }

    return true;
}

} // namespace srk