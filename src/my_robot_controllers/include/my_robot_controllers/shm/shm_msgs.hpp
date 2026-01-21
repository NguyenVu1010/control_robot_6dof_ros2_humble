#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_

#include <atomic>

namespace my_robot_controllers {
namespace shm {

const char* const ROBOT_SHM_NAME = "/robot_control_shm";

struct RobotData {
    // --- READ-ONLY (Robot gửi lên GUI) ---
    double joint_pos[6];
    double joint_vel[6];
    double ee_pos[3];    // x, y, z
    double ee_rpy[3];    // roll, pitch, yaw

    // --- WRITE (GUI gửi xuống Robot) ---
    
    // 1. Target Pose (Thay cho cmd_linear cũ)
    double target_pos[3]; // X, Y, Z
    
    // 2. Target Orientation (Thay cho cmd_angular cũ)
    double target_rpy[3]; // Roll, Pitch, Yaw
    
    // 3. Gripper Command
    double cmd_gripper;

    // Flags
    std::atomic<bool> cmd_active; 
    std::atomic<bool> sys_ready;
};

} // namespace shm
} // namespace my_robot_controllers

#endif