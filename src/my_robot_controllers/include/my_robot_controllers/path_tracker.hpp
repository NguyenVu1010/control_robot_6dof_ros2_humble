#pragma once
#include "simple_trajectory_lib/trajectory_generator.hpp"
#include <Eigen/Dense>
#include <memory>

// Giả sử srk::Frame là alias của Eigen::Isometry3d hoặc tương tự
#include "simple_kinematics_lib/kinematics_core.hpp" 

namespace my_robot_controllers {

class PathTracker {
public:
    PathTracker() {
        traj_gen_ = std::make_shared<stl::TrajectoryGenerator>();
        // Gain mặc định
        kp_pos_ = 15.0; kp_rot_ = 10.0;
        ki_pos_ = 0.1;  ki_rot_ = 0.0;
        resetError();
    }

    // Cài đặt quỹ đạo mới
    void setPath(const std::vector<stl::Frame>& waypoints, double duration) {
        traj_gen_->setPath(waypoints, duration);
        resetError();
    }

    void stop() {
        traj_gen_->stop();
        resetError();
    }

    bool isRunning() const {
        return traj_gen_->isRunning();
    }

    // Hàm tính toán cốt lõi: Closed-loop Control
    // Trả về: Cartesian Velocity (v_linear, w_angular)
    Eigen::VectorXd computeControlCommand(double dt, const srk::Frame& current_pose, srk::Frame& out_target_ghost) {
        Eigen::VectorXd v_cmd(6);
        v_cmd.setZero();
        
        srk::Frame target_step;
        srk::Vector6d v_ff; // Feed-forward velocity (vận tốc lý thuyết của quỹ đạo)

        if (traj_gen_->computeStep(dt, target_step, v_ff)) {
            // 1. Tính lỗi vị trí (Position Error)
            Eigen::Vector3d p_err = target_step.translation() - current_pose.translation();
            
            // 2. Tính lỗi góc (Orientation Error)
            Eigen::Quaterniond q_target(target_step.linear());
            Eigen::Quaterniond q_curr(current_pose.linear());
            Eigen::Quaterniond q_diff = q_target * q_curr.inverse();
            Eigen::AngleAxisd aa_err(q_diff);
            Eigen::Vector3d r_err = aa_err.axis() * aa_err.angle();

            // 3. Tích phân lỗi (Integral term - chống sai số tĩnh)
            integral_pos_ += p_err * dt;
            // Giới hạn tích phân (Anti-windup)
            double i_limit = 0.5; 
            integral_pos_ = integral_pos_.cwiseMin(i_limit).cwiseMax(-i_limit);

            // 4. Control Law: V_cmd = V_FF + Kp*Error + Ki*Integral
            v_cmd.head(3) = v_ff.head(3) + (p_err * kp_pos_) + (integral_pos_ * ki_pos_);
            v_cmd.tail(3) = v_ff.tail(3) + (r_err * kp_rot_); // Thường góc ít dùng Ki

            // Output trạng thái ảo để hiển thị lên GUI
            out_target_ghost = target_step;
        } else {
            // Đã hết quỹ đạo hoặc dừng
            v_cmd.setZero();
        }

        return v_cmd;
    }

    // Hàm để set PID gain động nếu cần
    void setGains(double kp_p, double ki_p, double kp_r) {
        kp_pos_ = kp_p; ki_pos_ = ki_p; kp_rot_ = kp_r;
    }

private:
    void resetError() {
        integral_pos_.setZero();
    }

    std::shared_ptr<stl::TrajectoryGenerator> traj_gen_;
    
    // Controller Gains
    double kp_pos_, kp_rot_;
    double ki_pos_, ki_rot_;
    
    // State variables
    Eigen::Vector3d integral_pos_;
};

} // namespace my_robot_controllers