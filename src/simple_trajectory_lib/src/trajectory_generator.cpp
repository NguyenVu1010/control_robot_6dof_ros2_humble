#include "simple_trajectory_lib/trajectory_generator.hpp"
#include <cmath>
#include <algorithm>

namespace stl {

TrajectoryGenerator::TrajectoryGenerator() 
    : is_running_(false), current_time_(0.0), total_duration_(0.0), current_segment_idx_(0) {}

void TrajectoryGenerator::setPath(const std::vector<Frame>& points, double duration) {
    if (points.size() < 2) return;
    
    waypoints_.clear();
    total_duration_ = (duration <= 0.0) ? 1.0 : duration;

    // Phân bổ thời gian dựa trên khoảng cách
    std::vector<double> segment_dists;
    double total_dist = 0.0;
    for (size_t i = 0; i < points.size() - 1; ++i) {
        double d = (points[i+1].translation() - points[i].translation()).norm();
        segment_dists.push_back(d);
        total_dist += d;
    }

    double acc_time = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
        Waypoint wp;
        wp.pose = points[i];
        if (i > 0 && total_dist > 1e-6) {
            acc_time += (segment_dists[i-1] / total_dist) * total_duration_;
        }
        wp.time = acc_time;
        wp.velocity.setZero();
        waypoints_.push_back(wp);
    }

    current_time_ = 0.0;
    current_segment_idx_ = 0;
    is_running_ = true;
}

double TrajectoryGenerator::interpolateCubic(double p0, double p1, double v0, double v1, double T, double t, double& vel_out) {
    if (T < 1e-6) return p1;
    double a0 = p0;
    double a1 = v0;
    double a2 = (3.0*(p1 - p0) - (2.0*v0 + v1)*T) / (T*T);
    double a3 = (2.0*(p0 - p1) + (v0 + v1)*T) / (T*T*T);
    vel_out = a1 + 2.0*a2*t + 3.0*a3*t*t;
    return a0 + a1*t + a2*t*t + a3*t*t*t;
}

bool TrajectoryGenerator::computeStep(double dt, Frame& pose_out, Vector6d& v_out) {
    if (!is_running_) return false;

    current_time_ += dt;
    if (current_time_ >= total_duration_) {
        pose_out = waypoints_.back().pose;
        v_out.setZero();
        is_running_ = false;
        return true;
    }

    while (current_segment_idx_ < waypoints_.size() - 2 && 
           current_time_ > waypoints_[current_segment_idx_ + 1].time) {
        current_segment_idx_++;
    }

    const auto& wp0 = waypoints_[current_segment_idx_];
    const auto& wp1 = waypoints_[current_segment_idx_ + 1];
    double T = wp1.time - wp0.time;
    double t = current_time_ - wp0.time;

    // Nội suy vị trí
    for (int i = 0; i < 3; ++i) {
        pose_out.translation()(i) = interpolateCubic(
            wp0.pose.translation()(i), wp1.pose.translation()(i),
            wp0.velocity(i), wp1.velocity(i), T, t, v_out(i));
    }

    // Nội suy hướng (Slerp smoothed by cubic)
    double s = std::clamp(t / T, 0.0, 1.0);
    double s_cubic = 3.0*s*s - 2.0*s*s*s;
    double ds_cubic = (6.0*s - 6.0*s*s) / T;

    Eigen::Quaterniond q0(wp0.pose.rotation());
    Eigen::Quaterniond q1(wp1.pose.rotation());
    pose_out.linear() = q0.slerp(s_cubic, q1).toRotationMatrix();

    Eigen::AngleAxisd aa(q1 * q0.inverse());
    v_out.tail(3) = aa.axis() * (aa.angle() * ds_cubic);

    return true;
}

} // namespace stl