#include <Arduino.h>
#include "Behaviors.h"
#include <Romi32U4.h>
#include "IR_sensor.h"
#include "Encoders.h"
#include "IMU.h"

Behaviors money;
IMU_sensor startGyro;


void setup() {
  money.Init();
}

void loop() {
  money.Run();
}