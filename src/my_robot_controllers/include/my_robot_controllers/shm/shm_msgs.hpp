#ifndef MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_
#define MY_ROBOT_CONTROLLERS__SHM__SHM_MSGS_HPP_

namespace my_robot_controllers {
namespace shm {

const char* const ROBOT_SHM_NAME = "/robot_control_shm";
const int MAX_WAYPOINTS = 10;

// Sửa tên Enum ở đây để khớp với GUI Python (MODE_POSE, MODE_TRAJ, MODE_JOINT)
enum ControlMode { 
    MODE_IDLE = 0, 
    MODE_POSE = 1,   // Thay cho MODE_CARTESIAN_POSE
    MODE_TRAJECTORY = 2,   // Thay cho MODE_TRAJECTORY
    MODE_JOINT = 3   // Thay cho MODE_JOINT_MANUAL
};

struct RobotData {
    // --- FEEDBACK ---
    double joint_pos[6]; double joint_vel[6];
    double ee_pos[3];    double ee_rpy[3];

    // --- COMMAND ---
    int control_mode;    
    int _padding1;        

    double target_pos[3]; 
    double target_rpy[3]; 
    double traj_vel_linear[3];
    double traj_vel_angular[3];

    int path_size;            
    int _padding2;            

    double path_points[MAX_WAYPOINTS][6]; 
    
    double traj_duration;     
    int traj_start_trigger;   
    int _padding3;            

    double manual_joint_vel[6]; 
    double cmd_gripper;

    bool cmd_active; 
    bool sys_ready;
};
}
}
#endif