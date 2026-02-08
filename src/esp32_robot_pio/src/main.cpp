#include <Arduino.h>
#include "hardware_manager.h"
#include "comm_manager.h"
#include "comm_manager.h"

hw_timer_t* baseTimer = NULL;
void setup() {
  Serial.begin(115200);
  initHardware();
  baseTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(baseTimer, &baseStepISR, true);
  timerAlarmWrite(baseTimer, BASE_TICK_US, true);
  timerAlarmEnable(baseTimer);
  xTaskCreatePinnedToCore(taskEncoder, "ENC_TASK", 4096, NULL, 3, NULL, 1);
}
void loop() {
  handleSerialCommunication();
  sendFeedback();
}
