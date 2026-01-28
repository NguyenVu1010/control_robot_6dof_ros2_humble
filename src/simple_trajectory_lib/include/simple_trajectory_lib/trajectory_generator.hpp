#ifndef SIMPLE_TRAJECTORY_LIB_HPP
#define SIMPLE_TRAJECTORY_LIB_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace stl {

using Frame = Eigen::Isometry3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class TrajectoryGenerator {
public:
    TrajectoryGenerator();

    void setPath(const Frame& start_pose, const Frame& end_pose, double duration);

    // CẬP NHẬT: Trả về cả Pose và Velocity
    bool computeStep(double dt, Frame& pose_out, Vector6d& v_out);

    void stop();
    bool isRunning() const { return is_running_; }

private:
    bool is_running_;
    double current_time_;
    double total_duration_;

    Frame start_pose_;   // Lưu điểm bắt đầu
    Vector6d diff_vector_; // Tổng quãng đường (3 linear, 3 angular axis)
};

}
#endif