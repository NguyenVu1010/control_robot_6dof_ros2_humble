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
    // --- READ-ONLY (Robot gửi lên) ---
    double joint_pos[6];
    double joint_vel[6];
    double ee_pos[3];
    double ee_rpy[3];

    // --- WRITE (GUI gửi xuống) ---
    
    // 1. Chế độ điều khiển (0, 1, 2, 3)
    int control_mode;

    // 2. Dữ liệu cho MODE 1 (Pose Control)
    double target_pos[3];
    double target_rpy[3];
    
    // 3. Dữ liệu cho MODE 2 (Trajectory Velocity Feed-forward)
    // GUI sẽ stream vận tốc mong muốn vào đây (vx, vy, vz, wx, wy, wz)
    double traj_vel_linear[3];
    double traj_vel_angular[3];

    // 4. Dữ liệu cho MODE 3 (Joint Manual)
    double manual_joint_vel[6]; 

    // 5. Kẹp (Dùng chung cho mọi mode)
    double cmd_gripper;

    std::atomic<bool> cmd_active; 
    std::atomic<bool> sys_ready;
};

} // namespace shm
} // namespace my_robot_controllers

#endif