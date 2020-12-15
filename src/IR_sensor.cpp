#include <Romi32U4.h>
#include "IR_sensor.h"
//#include <cmath>

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
}

float IRsensor::ReadData(void)
{
  //assignment 1.1
  //read out and calibrate your IR sensor, to convert readouts to distance in [cm]
  float ADCval = analogRead(pin_IR);
  float voltage = (5/1024.0)*ADCval;
  // float distance =0.0117*(pow(ADCval,2)) -2.2388*ADCval +194.8069;
  float distance = 21.239/voltage + 0.1605 ;
  //float distance = 20/voltage - 0.1;

  return distance;
}