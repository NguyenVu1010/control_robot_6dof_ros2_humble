#ifndef MY_ROBOT_HARDWARE__PROTOCOL_HPP_
#define MY_ROBOT_HARDWARE__PROTOCOL_HPP_

#include <cstdint>

namespace my_robot_hardware {

// Cấu trúc gửi xuống Robot (Command)
struct RobotCommand {
    uint16_t header = 0x55AA;
    float joint_velocities[6]; // 6 khớp tay (rad/s)
    float gripper_position;    // 1 khớp kẹp (0.0 - 1.0 hoặc rad)
    uint8_t checksum;
} __attribute__((packed));

// Cấu trúc đọc từ Robot (Feedback)
struct RobotFeedback {
    uint16_t header;           // Kỳ vọng 0xAA55
    float joint_positions[6];
    float joint_velocities[6];
    float gripper_position;
    uint8_t checksum;
} __attribute__((packed));

// Hàm tính checksum đơn giản
inline uint8_t calculate_checksum(const uint8_t* data, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) sum += data[i];
    return sum;
}

} 
#endif