#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_

#include <atomic>

namespace my_robot_controllers {
namespace shm {

const char* const ROBOT_SHM_NAME = "/robot_control_shm";

enum ControlMode {
    MODE_IDLE = 0,
    MODE_CARTESIAN_POSE = 1,
    MODE_TRAJECTORY = 2,
    MODE_JOINT_MANUAL = 3
};

struct RobotData {
    // --- PHẦN 1: FEEDBACK (Robot -> GUI) - Offset 0 ---
    double joint_pos[6];      // 0   -> 48
    double joint_vel[6];      // 48  -> 96
    double ee_pos[3];         // 96  -> 120 (X, Y, Z)
    double ee_rpy[3];         // 120 -> 144 (Roll, Pitch, Yaw)

    // --- PHẦN 2: COMMAND (GUI -> Robot) - Offset 144 ---
    int control_mode;         // 144 -> 148
    int _padding1;            // 148 -> 152 (Ép địa chỉ tiếp theo chia hết cho 8)

    double target_pos[3];     // 152 -> 176
    double target_rpy[3];     // 176 -> 200
    
    double traj_vel_linear[3];  // 200 -> 224
    double traj_vel_angular[3]; // 224 -> 248

    double traj_duration;       // 248 -> 256
    
    int traj_start_trigger;     // 256 -> 260
    int _padding2;              // 260 -> 264

    double manual_joint_vel[6]; // 264 -> 312
    double cmd_gripper;         // 312 -> 320

    // Flags (Atomic chiếm 1 byte mỗi biến trong memory nhưng thường được align)
    std::atomic<bool> cmd_active; // 320
    std::atomic<bool> sys_ready;  // 321
};

} // namespace shm
} // namespace my_robot_controllers
#endif