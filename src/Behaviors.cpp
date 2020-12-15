#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"
#include "Position_estimation.h"
#include "Rangefinder.h"
#include "Speed_controller.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"

//sensors
Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
IMU_sensor LSM6;
//IRsensor SharpIR;

extern Rangefinder rangefinder;

//motor-speed controller
SpeedController PIcontroller;

WallFollowingController PDcontroller;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectCollision2(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > 400) || (abs(data[1]) > 400)) return 1;
    else return 0;
}

boolean Behaviors::DetectRamp(void)
{
    auto data_acc = LSM6.ReadG();
    data[0] = med_x.Filter(data_acc.X);
    data[1] = med_y.Filter(data_acc.Y);
    data[2] = med_z.Filter(data_acc.Z);
long diffZ = abs(startZ - (data[2]));
if (diffZ > 10580) return 1;
   // if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}


boolean Behaviors::DetectCollision3(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
   // long mag = sqrt((data[0])^2 + (data[1])^2); 
    if //(abs(data[2]) < 400) &&
    
     ((abs(data[0]) > 330) || (abs(data[1]) > 330))
      return 1;
    else return 0;
}




boolean Behaviors::DetectBeingPickedUp(void)
{
    //assignment 2
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[2]) > threshold_pick_up)) return 1;
    else return 0;
}

boolean Behaviors:: CloseToWall(void){
    double sensorRead = PDcontroller.DistanceFromWall();

if (sensorRead<10){
    return true;
} else {
return false; }
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void)
{
    switch (robot_state)
    {
        // Serial.println(robot_state);

    case IDLE:
    {
        if(buttonA.getSingleDebouncedRelease()){ 
            delay(1000);
            robot_state = DRIVEWALL1; 
            PIcontroller.Stop(); 
        } 
        else { 
            robot_state = IDLE; 
            PIcontroller.Stop(); 
        }   
        break;
    }
    case DRIVEWALL1:
    {
    //   auto data_acc = LSM6.ReadG();
    // startX = (data_acc.X);
    // startY = (data_acc.Y);
    // startZ = (data_acc.Z);
    if (buttonA.getSingleDebouncedRelease()) 
    {
        robot_state = IDLE;
        PIcontroller.Stop(); 

    } if (DetectCollision()){
        robot_state = IDLEWALL1;
        PIcontroller.Stop();      
       
    }  else robot_state = DRIVEWALL1;
            PIcontroller.Run(100,100);
            break;
    }

    case IDLEWALL1:
    {
        if(buttonA.getSingleDebouncedRelease()){ 
         //delay(1000);
            robot_state = REVERSEWALL1; 
             
        } 
        else { 
            robot_state = IDLEWALL1; 
            PIcontroller.Stop(); 
        }   
        break;
    }

    case REVERSEWALL1:
    {
    PIcontroller.Reverse(100,14); 
    //PIcontroller.MoveToPosition(-0.2,0);
    robot_state = TURNWALL1; 
    break;
    }
        

    case TURNWALL1:
    {
    PIcontroller.Turn(50,0);
    //delay(500);
    robot_state = DRIVEWALL2;
    break;
    }
       

    case DRIVEWALL2:
    {
    if (DetectCollision3()) { //PDcontroller.DistanceFromWall() <= 15.0
        //delay(200);
        //PIcontroller.Straight(110,3);
        robot_state = UPRAMP;
       // PIcontroller.Stop();
       // robot_state = IDLE;
    } else {
        //robot_state = DRIVEWALL2;
        int speed = PDcontroller.Process(37); //distance in [cm]
         PIcontroller.Process(90-speed,90+speed);
    } 
    break;
    }   
        
    case UPRAMP:
    {
     if (DetectCollision2()) {
        PIcontroller.Straight(50,2);
        PIcontroller.Turn(90,0);
        robot_state = IDLE;
        PIcontroller.Stop();
    } else {
        robot_state = UPRAMP;
        int speed = PDcontroller.Process(35); //distance in [cm]
         PIcontroller.Process(145-speed,145+speed);
    } 
    break;
    }
    
    }    
   Serial.println(robot_state);
}