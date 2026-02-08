#include "comm_manager.h"
#include "hardware_manager.h"

uint8_t calculateChecksum(uint8_t* data, size_t len) {
  uint8_t sum = 0; 
  for (size_t i = 0; i < len; i++) sum += data[i]; 
  return sum;
}

void handleSerialCommunication() {
  // Kiểm tra nếu có đủ số byte của RobotCommand
  if (Serial.available() >= sizeof(RobotCommand)) {
    // Tìm byte đầu tiên là 0xAA (Khớp với ptr[0] của cmd.header = 0x55AA trên PC)
    if (Serial.peek() == 0xAA) {
      RobotCommand cmd;
      Serial.readBytes((uint8_t*)&cmd, sizeof(RobotCommand));

      uint8_t calc = calculateChecksum((uint8_t*)&cmd, sizeof(RobotCommand) - 1);
      
      // Header check: 0x55AA (Little Endian trên ESP32 sẽ khớp giá trị này)
      if (calc == cmd.checksum && cmd.header == 0x55AA) {
        for (int i = 0; i < 6; i++) {
          float rad_s = cmd.joint_velocities[i];
          float steps_per_sec = fabs(rad_s * joints[i].gearRatio * STEPS_PER_RAD);
          
          // Set chiều quay
          digitalWrite(joints[i].dirPin, (rad_s >= 0) ^ (joints[i].motorSign < 0));
          
          portENTER_CRITICAL(&joints[i].mux);
          if (steps_per_sec < 0.5f) {
            joints[i].enable = false;
          } else {
            joints[i].stepInterval = (uint32_t)(1000000.0f / steps_per_sec);
            joints[i].enable = true;
          }
          portEXIT_CRITICAL(&joints[i].mux);
        }
        setServoAngle(cmd.gripper_position);
      }
    } else {
      Serial.read(); // Bỏ byte không khớp để quét byte tiếp theo
    }
  }
}

void sendFeedback() {
  static uint32_t lastFB = 0;
  if (millis() - lastFB >= 20) { // 50Hz
    RobotFeedback fb;
    // PC mong đợi 0xAA trước rồi 0x55 -> Gán header = 0x55AA
    fb.header = 0x55AA; 
    
    for (int i = 0; i < 6; i++) {
      portENTER_CRITICAL(&joints[i].mux);
      fb.joint_positions[i] = joints[i].filtAngleRad;
      portEXIT_CRITICAL(&joints[i].mux);
      fb.joint_velocities[i] = 0; // Backend PC chưa dùng nên để 0
    }
    fb.gripper_position = current_gripper_pos;
    fb.checksum = calculateChecksum((uint8_t*)&fb, sizeof(RobotFeedback) - 1);
    
    Serial.write((uint8_t*)&fb, sizeof(RobotFeedback));
    lastFB = millis();
  }
}