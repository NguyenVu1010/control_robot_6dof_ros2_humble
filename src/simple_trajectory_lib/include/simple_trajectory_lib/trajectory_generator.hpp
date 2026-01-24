#ifndef SIMPLE_TRAJECTORY_LIB_HPP
#define SIMPLE_TRAJECTORY_LIB_HPP

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

namespace stl { // stl = Simple Trajectory Lib

using Frame = Eigen::Isometry3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

class TrajectoryGenerator {
public:
    TrajectoryGenerator();

    // Thiết lập hành trình từ A -> B trong thời gian duration (giây)
    void setPath(const Frame& start_pose, const Frame& end_pose, double duration);

    // Tính toán vận tốc tại thời điểm delta_time (dt) trôi qua
    // Trả về false nếu đã đi hết hành trình
    bool computeStep(double dt, Vector6d& v_out);

    // Dừng hoặc Reset
    void stop();
    bool isRunning() const { return is_running_; }

private:
    bool is_running_;
    double current_time_;
    double total_duration_;

    // Vector sai lệch tổng (Linear & Angular)
    Vector6d diff_vector_; 
};

}
#endif