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
// SVD Per-Axis Damping: chỉ damping hướng gần singularity, giữ nguyên hướng tốt
bool KinematicsCore::solveIK_Velocity(const JntArray& q, const Vector6d& v_cart, JntArray& q_dot_out) {
    if (!initialized_ || q.size() != n_joints_) return false;

    // 1. Tính Jacobian
    Jacobian J(6, n_joints_);
    internal_compute_jacobian(q, J);

    // 2. SVD phân rã: J = U * Σ * V^T
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto& S = svd.singularValues();  // σ₁ ≥ σ₂ ≥ ... ≥ σₙ
    const auto& U = svd.matrixU();
    const auto& V = svd.matrixV();

    int rank = S.size();

    // 3. Per-axis damped pseudo-inverse: q̇ = Σᵢ (σᵢ / (σᵢ² + λᵢ²)) * (uᵢᵀ · v_cart) * vᵢ
    q_dot_out.resize(n_joints_);
    q_dot_out.setZero();

    for (int i = 0; i < rank; ++i) {
        double sigma = S(i);
        double lambda_i = 0.0;

        // Quadratic ramp: hướng có σ nhỏ → λ tăng dần
        if (sigma < ik_config_.sigma_threshold) {
            double ratio = 1.0 - sigma / ik_config_.sigma_threshold;
            lambda_i = ik_config_.lambda_max * ratio * ratio;
        }

        double factor = sigma / (sigma * sigma + lambda_i * lambda_i);
        double alpha = U.col(i).dot(v_cart) * factor;
        q_dot_out += alpha * V.col(i);
    }

    // 4. Direction Guard: hậu kiểm hướng thực tế so với mong muốn
    double scale = computeDirectionGuardScale(J, v_cart, q_dot_out);
    if (scale < 1.0) {
        q_dot_out *= scale;
    }

    // 5. Safety clamp (Giới hạn tốc độ khớp)
    for (unsigned int i = 0; i < n_joints_; ++i) {
        q_dot_out(i) = std::clamp(q_dot_out(i), -ik_config_.max_joint_speed, ik_config_.max_joint_speed);
    }

    return true;
}

// --- DIRECTION GUARD ---
// Kiểm tra cos(v_desired, J·q̇), nếu sai hướng → scale down
double KinematicsCore::computeDirectionGuardScale(const Jacobian& J, const Vector6d& v_desired, const JntArray& q_dot) {
    double v_norm = v_desired.norm();
    if (v_norm < 1e-9) return 1.0;  // Không có lệnh → bỏ qua

    Vector6d v_actual = J * q_dot;
    double va_norm = v_actual.norm();
    if (va_norm < 1e-9) return 1.0;

    double cos_angle = v_desired.dot(v_actual) / (v_norm * va_norm);

    if (cos_angle >= ik_config_.direction_cos_threshold) {
        return 1.0;  // Hướng đúng → giữ nguyên
    }

    // Scale tuyến tính từ 1.0 xuống direction_min_scale khi cos giảm từ threshold xuống 0
    double t = std::max(0.0, cos_angle) / ik_config_.direction_cos_threshold;
    return ik_config_.direction_min_scale + (1.0 - ik_config_.direction_min_scale) * t;
}

// --- INVERSE KINEMATICS (POSITION LEVEL) ---
// Dùng phương pháp Newton-Raphson Iterative
bool KinematicsCore::solveIK_Position(const JntArray& q_current, const Frame& target, JntArray& q_dot_out) {
    if (!initialized_) return false;

    // 1. Tính toán vị trí hiện tại (FK)
    Frame current_pose;
    solveFK(q_current, current_pose);

    // 2. Tính Vector Sai Số (Error Twist e)
    // e = Target - Current
    Eigen::Vector3d p_err = target.translation() - current_pose.translation();
    
    Eigen::Quaterniond q_cur_R(current_pose.linear());
    Eigen::Quaterniond q_tar_R(target.linear());
    q_cur_R.normalize();
    q_tar_R.normalize();
    
    // Sai số góc: q_diff = q_target * q_current^-1
    Eigen::Quaterniond q_diff = q_tar_R * q_cur_R.inverse();
    Eigen::AngleAxisd aa(q_diff);
    Eigen::Vector3d w_err = aa.axis() * aa.angle();

    // 3. P-Controller: Biến sai số thành Vận tốc mong muốn (V_cmd)
    // V_cmd = Kp * Error
    Vector6d v_cmd;
    v_cmd.head(3) = p_err * ik_config_.Kp_pos;
    v_cmd.tail(3) = w_err * ik_config_.Kp_rot;

    // Kẹp vận tốc Cartesian tối đa (để không bị giật khi sai số lớn)
    double lin_norm = v_cmd.head(3).norm();
    if (lin_norm > ik_config_.max_lin_vel) {
        v_cmd.head(3) *= (ik_config_.max_lin_vel / lin_norm);
    }
    double rot_norm = v_cmd.tail(3).norm();
    if (rot_norm > ik_config_.max_rot_vel) {
        v_cmd.tail(3) *= (ik_config_.max_rot_vel / rot_norm);
    }

    // 4. Giải bài toán vận tốc: q_dot = J_pinv * V_cmd
    // Sử dụng lại logic SVD Damping để đảm bảo mượt mà
    return solveIK_Velocity(q_current, v_cmd, q_dot_out);
}

// --- IK CONFIG ---
void KinematicsCore::setIKConfig(const IKConfig& config) { ik_config_ = config; }
const IKConfig& KinematicsCore::getIKConfig() const { return ik_config_; }

} // namespace srk