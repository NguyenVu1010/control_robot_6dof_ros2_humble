#ifndef TRAJECTORY_GENERATOR_HPP
#define TRAJECTORY_GENERATOR_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <vector>

namespace stl {

using Frame = Eigen::Isometry3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

struct Waypoint {
    Frame pose;
    double time;
    Vector6d velocity;
};

class TrajectoryGenerator {
public:
    TrajectoryGenerator();
    void setPath(const std::vector<Frame>& points, double duration);
    bool computeStep(double dt, Frame& pose_out, Vector6d& v_out);
    void stop() { is_running_ = false; }
    bool isRunning() const { return is_running_; }

private:
    double interpolateCubic(double p0, double p1, double v0, double v1, double T, double t, double& vel_out);

    bool is_running_;
    double current_time_;
    double total_duration_;
    std::vector<Waypoint> waypoints_;
    size_t current_segment_idx_;
};

} // namespace stl
#endif