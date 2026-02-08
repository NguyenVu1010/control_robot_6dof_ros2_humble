#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <Arduino.h>

#pragma pack(push, 1)
struct RobotCommand {
  uint16_t header;           // PC gửi 0x55AA -> ESP32 nhận [0xAA, 0x55]
  float joint_velocities[6]; 
  float gripper_position;    
  uint8_t checksum;
};

struct RobotFeedback {
  uint16_t header;           // ESP32 gửi 0x55AA -> PC nhận [0xAA, 0x55]
  float joint_positions[6];  
  float joint_velocities[6]; 
  float gripper_position;    
  uint8_t checksum;
};
#pragma pack(pop)

uint8_t calculateChecksum(uint8_t* data, size_t len);
#endif