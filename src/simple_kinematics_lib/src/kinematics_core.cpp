#include "simple_kinematics_lib/kinematics_core.hpp"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <eigen3/Eigen/SVD>
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

    JntArray q_sol = q_current;
    Frame current_pose;
    Jacobian J(6, n_joints_);
    
    // Tăng số lần lặp để đảm bảo hội tụ chính xác
    int max_iter = 500;
    double eps_pos = 1e-6; // Độ chính xác 1 micromet
    double eps_rot = 1e-5;

    for (int iter = 0; iter < max_iter; ++iter) {
        solveFK(q_sol, current_pose);
        
        // 1. Tính sai số
        Eigen::Vector3d p_err = target.translation() - current_pose.translation();
        
        Eigen::Quaterniond q_cur_R(current_pose.linear());
        Eigen::Quaterniond q_tar_R(target.linear());
        q_cur_R.normalize();
        q_tar_R.normalize();
        
        Eigen::Quaterniond q_diff = q_tar_R * q_cur_R.inverse();
        Eigen::AngleAxisd aa(q_diff);
        Eigen::Vector3d w_err = aa.axis() * aa.angle();

        Vector6d err_vec;
        err_vec << p_err, w_err;

        if (p_err.norm() < eps_pos && w_err.norm() < eps_rot) break; 

        // 2. Tính Jacobian
        internal_compute_jacobian(q_sol, J);

        // 3. SVD VỚI DAMPING MƯỢT (Smooth Damped SVD)
        // Đây là bí quyết để hết rung: Không cắt bỏ đột ngột
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
        
        Eigen::VectorXd singularValues = svd.singularValues();
        Eigen::VectorXd singularValuesInv = singularValues;

        // Hệ số damping cực nhỏ để không ảnh hưởng độ chính xác trong vùng làm việc
        double lambda = 1e-4; 

        for (int i = 0; i < singularValues.size(); ++i) {
            double s = singularValues(i);
            
            // Công thức Damped SVD: s / (s^2 + lambda^2)
            // - Khi s lớn (vùng an toàn): lambda^2 không đáng kể -> s/s^2 = 1/s (Nghịch đảo chuẩn -> Chính xác tuyệt đối)
            // - Khi s nhỏ (vùng biên): lambda^2 chiếm ưu thế -> s tiến về 0 một cách mượt mà -> Không rung
            singularValuesInv(i) = s / (s * s + lambda * lambda);
        }

        // Tính bước nhảy
        JntArray delta_q = svd.matrixV() * singularValuesInv.asDiagonal() * svd.matrixU().transpose() * err_vec;

        // 4. SCALING BƯỚC NHẢY (Thay vì Clamping)
        // Giữ nguyên hướng di chuyển, chỉ giảm độ lớn nếu quá to
        double max_step = 0.2; // Giảm xuống 0.2 để ổn định hơn ở biên
        if (delta_q.norm() > max_step) {
            delta_q = delta_q * (max_step / delta_q.norm());
        }
        
        q_sol += delta_q;
    }

    // 5. TÍNH VẬN TỐC VỚI GAIN ĐỘNG (DYNAMIC GAIN)
    // Khi robot ở tư thế khó (Singularity), ta tự động giảm Gain xuống để nó vào khớp nhẹ nhàng
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_final(J, 0); // Tính SVD nhanh chỉ lấy giá trị kỳ dị
    double min_singular_value = svd_final.singularValues().minCoeff();
    
    double Kp_base = 20.0;
    
    // Nếu gần biên (min_singular_value nhỏ), giảm Gain xuống
    if (min_singular_value < 0.1) {
        Kp_base = 20.0 * (min_singular_value / 0.1); 
        if (Kp_base < 1.0) Kp_base = 1.0; // Tối thiểu
    }

    q_dot_out = (q_sol - q_current) * Kp_base;

    // 6. Safety Clamp
    double max_speed = 3.0;
    for(int i=0; i<n_joints_; ++i) {
        if(q_dot_out(i) > max_speed) q_dot_out(i) = max_speed;
        if(q_dot_out(i) < -max_speed) q_dot_out(i) = -max_speed;
    }

    return true;
}
} // namespace srk