#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"


IRsensor SharpIR;

void WallFollowingController::Init(void)
{
    SharpIR.Init();
}

float WallFollowingController::Process(float target_distance)
{
  //assignment 2: write a PD controller that outputs speed as a function of distance error
  distance = SharpIR.ReadData();
  E_distance = distance - target_distance;
  deriv = (prev_e_distance-E_distance);
  float speed = Kp*E_distance + Kd*deriv;
  prev_e_distance = distance;

  if(E_distance < 0){
    speed = speed;
  } else speed = -speed;

  return speed;
}

void WallFollowingController::Numbers(){
  distance = SharpIR.ReadData();
  Serial.println(distance);
}

double WallFollowingController::DistanceFromWall(){
  double measurement = SharpIR.ReadData();
  return measurement;
}

void WallFollowingController::PrintTime(){
  time = SharpIR.ReadData();
  Serial.println(time);
}

void WallFollowingController::PrintDistance(){
  time = SharpIR.ReadData();
  distance_s = (0.0177*time) - 0.3814;
  Serial.println(distance_s);
}