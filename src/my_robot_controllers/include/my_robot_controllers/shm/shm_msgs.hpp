#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_

#include <atomic>

namespace my_robot_controllers {
namespace shm {

const char* const ROBOT_SHM_NAME = "/robot_control_shm";

struct RobotData {
    // --- READ-ONLY CHO GUI (Write bởi Robot) ---
    double joint_pos[6];
    double joint_vel[6];
    double ee_pos[3];    // x, y, z
    double ee_rpy[3];    // roll, pitch, yaw

    // --- WRITE BỞI GUI (Read-only cho Robot) ---
    double cmd_linear[3];
    double cmd_angular[3];

    std::atomic<bool> cmd_active; 
    std::atomic<bool> sys_ready;
};

} // namespace shm
} // namespace my_robot_controllers

#endif