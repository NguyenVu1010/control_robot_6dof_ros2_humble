#include "my_robot_controllers/algo/trajectory_solver.hpp"
#include <cmath>
#include <algorithm>

namespace my_robot_controllers {
namespace algo {

TrajectorySolver::TrajectorySolver() 
    : is_running_(false), current_time_(0.0), total_duration_(0.0) 
{
    v_out_.setZero();
}

void TrajectorySolver::setTrajectory(const KDL::Frame& start_pose, 
                                     const KDL::Frame& end_pose, 
                                     double duration) 
{
    start_pose_ = start_pose;
    end_pose_ = end_pose;
    total_duration_ = std::max(duration, 0.1); // Tối thiểu 0.1s để tránh chia cho 0
    current_time_ = 0.0;
    
    // Tính vector sai lệch tổng (Total Displacement)
    // KDL::diff trả về Twist đại diện cho vector đi từ Start -> End
    total_diff_ = KDL::diff(start_pose, end_pose);
    
    is_running_ = true;
}

bool TrajectorySolver::computeVelocity(double dt, Vector6d& v_out) {
    if (!is_running_) {
        v_out.setZero();
        return false;
    }

    current_time_ += dt;

    // Kiểm tra nếu hết thời gian
    if (current_time_ >= total_duration_) {
        v_out.setZero();
        is_running_ = false;
        return false;
    }

    // --- THUẬT TOÁN NỘI SUY CUBIC (S-CURVE ĐƠN GIẢN) ---
    // t_norm = t / T (từ 0 đến 1)
    double t_norm = current_time_ / total_duration_;
    
    // Hàm vị trí: s(t) = 3t^2 - 2t^3 (Đi từ 0 -> 1, đạo hàm tại 0 và 1 bằng 0)
    // Hàm vận tốc (đạo hàm của s): s'(t) = 6t - 6t^2
    // Vận tốc thực tế = Total_Displacement * s'(t_norm) / Duration
    
    double velocity_scale = (6.0 * t_norm - 6.0 * t_norm * t_norm) / total_duration_;

    // Tính vận tốc Cartesian
    v_out(0) = total_diff_.vel.x() * velocity_scale;
    v_out(1) = total_diff_.vel.y() * velocity_scale;
    v_out(2) = total_diff_.vel.z() * velocity_scale;
    v_out(3) = total_diff_.rot.x() * velocity_scale;
    v_out(4) = total_diff_.rot.y() * velocity_scale;
    v_out(5) = total_diff_.rot.z() * velocity_scale;

    return true;
}

} }