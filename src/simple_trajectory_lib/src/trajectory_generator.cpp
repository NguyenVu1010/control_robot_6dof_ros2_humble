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
    if (duration <= 0.001) duration = 0.1;
    
    start_pose_ = start_pose; // Lưu lại vị trí gốc
    total_duration_ = duration;
    current_time_ = 0.0;
    is_running_ = true;

    // 1. Tính sai lệch tuyến tính
    Eigen::Vector3d p_err = end_pose.translation() - start_pose.translation();

    // 2. Tính sai lệch góc dùng Angle-Axis (để nội suy tuyến tính góc dễ dàng hơn)
    Eigen::Quaterniond q_start(start_pose.rotation());
    Eigen::Quaterniond q_end(end_pose.rotation());
    Eigen::Quaterniond q_diff = q_end * q_start.inverse();
    
    Eigen::AngleAxisd aa(q_diff);
    // Lưu vector quay tổng thể: trục quay * góc quay
    Eigen::Vector3d r_err = aa.axis() * aa.angle();

    diff_vector_ << p_err, r_err;
}

bool TrajectoryGenerator::computeStep(double dt, Frame& pose_out, Vector6d& v_out) {
    if (!is_running_) {
        v_out.setZero();
        return false;
    }

    current_time_ += dt;

    if (current_time_ >= total_duration_) {
        // Trả về kết quả cuối cùng trước khi dừng
        // Ở đây có thể gán pose_out = end_pose_ nếu bạn lưu end_pose_
        v_out.setZero();
        is_running_ = false;
        return false;
    }

    // --- CUBIC INTERPOLATION ---
    double T = total_duration_;
    double t = current_time_;
    
    // 1. Vị trí nội suy s(t) chạy từ 0 -> 1
    // s(t) = 3*(t/T)^2 - 2*(t/T)^3
    double s_t = 3.0 * std::pow(t/T, 2) - 2.0 * std::pow(t/T, 3);
    
    // 2. Vận tốc nội suy s'(t)
    // s'(t) = (6*t/T^2) - (6*t^2/T^3)
    double v_scale = (6.0 * t / (T * T)) - (6.0 * t * t / (T * T * T));

    // --- TÍNH VẬN TỐC (FEED-FORWARD) ---
    v_out = diff_vector_ * v_scale;

    // --- TÍNH POSE MỤC TIÊU (TARGET POSE) ---
    pose_out = start_pose_;
    
    // Nội suy tịnh tiến
    pose_out.translation() += diff_vector_.head(3) * s_t;
    
    // Nội suy hướng (Rotation)
    // Lấy vector xoay tại thời điểm t = vector_xoay_tổng * s_t
    Eigen::Vector3d current_r_vec = diff_vector_.tail(3) * s_t;
    double angle = current_r_vec.norm();
    if (angle > 1e-9) {
        Eigen::AngleAxisd current_aa(angle, current_r_vec.normalized());
        pose_out.linear() = (current_aa * Eigen::Quaterniond(start_pose_.rotation())).toRotationMatrix();
    }

    return true;
}

void TrajectoryGenerator::stop() {
    is_running_ = false;
}

}