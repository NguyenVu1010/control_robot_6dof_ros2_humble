#ifndef CONFIG_H
#define CONFIG_H
#include <Arduino.h>
#define I2C_SDA 21
#define I2C_SCL 22
#define TCA_ADDR 0x70
#define AS5600_ADDR 0x36
#define STEPS_REV 200
#define MICROSTEP 8
#define STEPS_PER_RAD ((STEPS_REV * MICROSTEP) / (2.0f * PI))
#define BASE_TICK_US 100 
#define SERVO_PIN 5
#define SERVO_CH  7
#endif
