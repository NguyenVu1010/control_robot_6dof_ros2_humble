#include "simple_trajectory_lib/trajectory_generator.hpp"
#include <cmath>
#include <algorithm>

namespace stl {

TrajectoryGenerator::TrajectoryGenerator() 
    : is_running_(false), current_time_(0.0), total_duration_(0.0) 
{
    diff_vector_.setZero();
}

void TrajectoryGenerator::setPath(const Frame& start_pose, const Frame& end_pose, double duration) {
    if (duration <= 0.001) duration = 0.1; // Safety
    
    total_duration_ = duration;
    current_time_ = 0.0;
    is_running_ = true;

    // 1. Tính sai lệch tuyến tính (Linear Diff)
    Eigen::Vector3d p_err = end_pose.translation() - start_pose.translation();

    // 2. Tính sai lệch góc (Angular Diff) dùng Quaternion
    Eigen::Quaterniond q_start(start_pose.rotation());
    Eigen::Quaterniond q_end(end_pose.rotation());
    // Delta Q = End * Start^-1
    Eigen::Quaterniond q_diff = q_end * q_start.inverse();
    Eigen::AngleAxisd aa(q_diff);
    Eigen::Vector3d r_err = aa.axis() * aa.angle();

    // Lưu tổng quãng đường cần đi vào vector 6D
    diff_vector_ << p_err, r_err;
}

bool TrajectoryGenerator::computeStep(double dt, Vector6d& v_out) {
    if (!is_running_) {
        v_out.setZero();
        return false;
    }

    current_time_ += dt;

    if (current_time_ >= total_duration_) {
        v_out.setZero();
        is_running_ = false;
        return false;
    }

    // --- CUBIC INTERPOLATION (S-Curve Velocity) ---
    // t_norm chạy từ 0 -> 1
    double t = current_time_ / total_duration_;
    
    // Đạo hàm của hàm vị trí s(t) = 3t^2 - 2t^3
    // v_scale = s'(t) = (6t - 6t^2) / Duration
    double velocity_scale = (6.0 * t - 6.0 * t * t) / total_duration_;

    // Vận tốc = Tổng quãng đường * Tỷ lệ vận tốc
    v_out = diff_vector_ * velocity_scale;

    return true;
}

void TrajectoryGenerator::stop() {
    is_running_ = false;
}

}