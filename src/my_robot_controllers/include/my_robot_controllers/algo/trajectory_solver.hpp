#ifndef MY_ROBOT_CONTROLLERS__ALGO__TRAJECTORY_SOLVER_HPP_
#define MY_ROBOT_CONTROLLERS__ALGO__TRAJECTORY_SOLVER_HPP_

#include "types.hpp"
#include <kdl/frames.hpp>

namespace my_robot_controllers {
namespace algo {

class TrajectorySolver {
public:
    TrajectorySolver();

    // Thiết lập quỹ đạo mới
    // Nếu start_pose không được truyền (hoặc logic bên ngoài xử lý), robot sẽ đi từ vị trí hiện tại
    void setTrajectory(const KDL::Frame& start_pose, 
                       const KDL::Frame& end_pose, 
                       double duration);

    // Tính toán vận tốc tại thời điểm hiện tại (dt: thời gian trôi qua từ lần gọi trước)
    // Trả về false nếu đã đi hết quỹ đạo
    bool computeVelocity(double dt, Vector6d& v_out);

    bool isBusy() const { return is_running_; }
    void stop() { is_running_ = false; v_out_.setZero(); }

private:
    bool is_running_;
    double current_time_;
    double total_duration_;

    KDL::Frame start_pose_;
    KDL::Frame end_pose_;
    
    // Biến lưu tổng quãng đường cần đi (Linear & Angular Vector)
    KDL::Twist total_diff_; 
    
    Vector6d v_out_;
};

} }
#endif