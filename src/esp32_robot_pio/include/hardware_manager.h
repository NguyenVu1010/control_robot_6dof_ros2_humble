#ifndef HARDWARE_MANAGER_H
#define HARDWARE_MANAGER_H
#include <Arduino.h>
#include "config.h"

typedef struct {
  int stepPin; int dirPin; uint8_t tcaCh;
  int8_t encSign; int8_t motorSign; float gearRatio;
  volatile uint32_t stepInterval; volatile bool enable;
  uint32_t acc; bool stepHigh;
  float lastRaw; float totalAngleDeg; float filtAngleRad;
  portMUX_TYPE mux;
} Joint;

extern Joint joints[6];
extern portMUX_TYPE timerMux;
extern float current_gripper_pos;

void initHardware();
void IRAM_ATTR baseStepISR();
void taskEncoder(void* p);
void setServoAngle(float pos);
float readAS5600(uint8_t ch);
#endif
