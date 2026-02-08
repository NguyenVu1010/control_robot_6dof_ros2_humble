#include "hardware_manager.h"
#include <Wire.h>
#include "driver/gpio.h"

Joint joints[6] = {
  {14, 27, 0, +1, +1, 25.0f}, {26, 25, 1, +1, -1, 25.0f},
  {33, 32, 2, +1, -1, 25.0f}, {13, 23, 3, +1, -1, 25.0f},
  {18, 19, 4, +1, -1, 25.0f}, {16, 17, 5, +1, -1, 1.0f}
};

float current_gripper_pos = 0.0f;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

void initHardware() {
  Wire.begin(I2C_SDA, I2C_SCL); Wire.setClock(400000);
  for (int i = 0; i < 6; i++) {
    pinMode(joints[i].stepPin, OUTPUT); pinMode(joints[i].dirPin, OUTPUT);
    joints[i].mux = portMUX_INITIALIZER_UNLOCKED; joints[i].enable = false;
  }
  ledcSetup(SERVO_CH, 50, 16); ledcAttachPin(SERVO_PIN, SERVO_CH);
}

void IRAM_ATTR baseStepISR() {
  portENTER_CRITICAL_ISR(&timerMux);
  for (int i = 0; i < 6; i++) {
    if (!joints[i].enable || joints[i].stepInterval == 0) continue;
    joints[i].acc += BASE_TICK_US;
    if (joints[i].acc >= joints[i].stepInterval) {
      joints[i].acc = 0; gpio_set_level((gpio_num_t)joints[i].stepPin, 1); joints[i].stepHigh = true;
    } else if (joints[i].stepHigh) {
      gpio_set_level((gpio_num_t)joints[i].stepPin, 0); joints[i].stepHigh = false;
    }
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void taskEncoder(void* p) {
  for (int i = 0; i < 6; i++) joints[i].lastRaw = readAS5600(joints[i].tcaCh);
  while (1) {
    for (int i = 0; i < 6; i++) {
      float a = readAS5600(joints[i].tcaCh); if (a < 0) continue;
      float d = (a - joints[i].lastRaw);
      if (d > 180) d -= 360; if (d < -180) d += 360;
      portENTER_CRITICAL(&joints[i].mux);
      joints[i].totalAngleDeg += d * joints[i].encSign;
      joints[i].filtAngleRad = (joints[i].totalAngleDeg * (PI / 180.0f)) / joints[i].gearRatio;
      portEXIT_CRITICAL(&joints[i].mux);
      joints[i].lastRaw = a;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

float readAS5600(uint8_t ch) {
  Wire.beginTransmission(0x70); Wire.write(1 << ch); Wire.endTransmission();
  Wire.beginTransmission(0x36); Wire.write(0x0C); Wire.endTransmission(false);
  Wire.requestFrom(0x36, 2);
  if (Wire.available() == 2) return ((Wire.read() << 8) | Wire.read() & 0x0FFF) * 0.087890625f;
  return -1;
}

void setServoAngle(float pos) {
  current_gripper_pos = pos; // Cập nhật vị trí hiện tại
  float deg = pos * 180.0f; // Giả sử ROS gửi 0.0-1.0
  uint32_t duty = (uint32_t)(((deg / 180.0f) * 2000 + 500) / 20000.0f * 65535.0f);
  ledcWrite(SERVO_CH, duty);
}
