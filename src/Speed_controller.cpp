#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}

void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right); //this is where your newly programmed function is/will be called
    }
}

boolean SpeedController::Turn(int degree, int direction)
{
    motors.setEfforts(0, 0);   
   // int turns = counts*(degree/180.0); //assignment 1: convert degree into counts
   int turns = counts*(degree/180.0);
   // int turns = 2*degree*(147/35);
    int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    {
        if(!direction) Run(45,-45);
        else Run(-45,45);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Straight(int target_velocity, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

 boolean SpeedController::MoveToPosition(float target_x, float target_y)
{    do {    
      //assignment
        float error_x = target_x - odometry.ReadPose().X;
        float error_y = target_y - odometry.ReadPose().Y;

        error_distance = sqrt(pow(error_x, 2) + pow(error_y, 2));
        float error_distance_acc =+ error_distance;
        
        error_theta = atan2(target_y - odometry.ReadPose().Y, target_x - odometry.ReadPose().X) - odometry.ReadPose().THETA;

        if (error_theta > (PI / 180) * 185) {
            error_theta -= 2 * PI;
        }
        else if (error_theta < -(PI / 180) * 185){
            error_theta += 2 * PI;
        }
        
        float error_theta_acc =+ error_theta;
        float Kp1 = 200;
        float Kp2 = 100;
        float Ki1 = 100;
        float Ki2 = 50;
      //  float Ki1 = 0;
        // float Ki2 = 0;

    //     float time = millis();
    //     if(time - prev_time >= 50) //update every 50ms for practical reasons
    // {
    //     float d_time = (time_track - prev_time)/1000.0; //convert to s
    //     float velocity_left = MagneticEncoder.ReadVelocityLeft()/1000.0; //in m/s
    //     float velocity_right = MagneticEncoder.ReadVelocityRight()/1000.0; //in m/s
    //     velocity = (velocity_left + velocity_right)/2.0; //in m/s
    //     acceleration = (velocity - prev_vel);//(50/1000); 
    //     prev_time = time;
    //     prev_vel = velocity;
        
    // }
    // float speedleft = constrain(Kp1 * error_distance - Kp2 * error_theta + Ki1 * error_distance_acc - Ki2 * error_theta_acc, -50, 50);
     //float speedright = constrain(Kp1 * error_distance + Kp2 * error_theta + Ki1 * error_distance_acc + Ki2 * error_theta_acc, -50, 50);
         float speedleft = constrain(Kp1 * error_distance - Kp2 * error_theta + Ki1 * error_distance_acc - Ki2 * error_theta_acc, velocity -100,velocity + 100);
        float speedright = constrain(Kp1 * error_distance + Kp2 * error_theta + Ki1 * error_distance_acc + Ki2 * error_theta_acc, velocity -100,velocity + 100);
        Run(speedleft, speedright);

        Serial.print(target_x);
        Serial.print('\t');
        Serial.print(odometry.ReadPose().X);
        Serial.print('\t');
        Serial.print(target_y);
        Serial.print('\t');
        Serial.print(odometry.ReadPose().Y);
        Serial.print('\t');
        Serial.print(speedleft);
        Serial.print('\t');
        Serial.print(speedright);
        Serial.print('\t');
        Serial.print(error_theta);
        Serial.print('\t');
        Serial.println(error_distance);

    } while (error_distance >= 0.03 ); //define a distance criteria that lets the robot know that it reached the waypoint.
    return 1;
}


boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Process(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();

        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        
        Serial.print(MagneticEncoder.ReadVelocityLeft());
        Serial.print('\t');
        Serial.println(MagneticEncoder.ReadVelocityRight());
    }
}

boolean SpeedController::Reverse(int target_velocity, int distance) //in mm/s and cm
{
    motors.setEfforts(0, 0);
    
    uint32_t duration = 1000*((distance*10)/(float)target_velocity); //in ms
    unsigned long now = millis();

    //Serial.print(duration);
    //Serial.print('\t');
    //Serial.println(now);

    while ((unsigned long)(millis() - now) <= duration){
        Run(-target_velocity,-target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
}