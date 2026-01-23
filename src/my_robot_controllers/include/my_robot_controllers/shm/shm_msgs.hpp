#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_

#include <atomic>

namespace my_robot_controllers {
namespace shm {

const char* const ROBOT_SHM_NAME = "/robot_control_shm";

// Định nghĩa các chế độ điều khiển
enum ControlMode {
    MODE_IDLE = 0,          // Dừng robot
    MODE_CARTESIAN_POSE = 1, // Điều khiển vị trí (P-Controller cũ)
    MODE_TRAJECTORY = 2,     // Chạy theo profile vận tốc (Path Generator)
    MODE_JOINT_MANUAL = 3    // Điều khiển vận tốc từng khớp bằng tay
};

struct RobotData {
    // Read-only (144 bytes) - Giữ nguyên
    double joint_pos[6];
    double joint_vel[6];
    double ee_pos[3];
    double ee_rpy[3];

    // Write (Bắt đầu từ 144)
    int control_mode;    // 144
    int _padding;        // 148

    // --- DỮ LIỆU ĐIỀU KHIỂN ---
    
    // 1. Target Pose (Dùng chung cho cả Mode Pose và Trajectory Point-to-Point)
    double target_pos[3]; // 152
    double target_rpy[3]; // 176
    
    // 2. Trajectory Params
    double traj_duration; // 200 (Thời gian di chuyển, ví dụ 5.0 giây)
    int traj_start_trigger; // 208 (GUI tăng số này lên để báo hiệu bắt đầu chạy)

    // 3. Manual Joints (Để điều khiển khớp tay)
    double manual_joint_vel[6]; // 216

    // 4. Gripper
    double cmd_gripper; // 264

    std::atomic<bool> cmd_active; // 272
    std::atomic<bool> sys_ready;  // 273
};

} // namespace shm
} // namespace my_robot_controllers

#endif