#include  "Position_estimation.h"
#include "Encoders.h"

Encoder RomiEncoders;
float x = 0;
float y = 0;
float theta = 0;
unsigned long time_prev = millis();
unsigned long time_now = 0;

void Position::Init(void)
{
    time_prev = millis();
    x = 0;
    y = 0;
    theta = 0;
}

void Position::Stop(void)
{
    time_prev = millis();
    x = 0; 
    y = 0;
    theta = 0;
}

Position::pose_data Position::ReadPose(void)
{
    return {x,y,theta};
}

void Position::PrintPose(void)
{
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(theta);
}

void Position::UpdatePose(float target_speed_left, float target_speed_right)
{
    // time_now = millis();
    // float V_r = RomiEncoders.ReadVelocityRight();
    // float V_l = RomiEncoders.ReadVelocityLeft();
    // float V = (V_l+V_r)/2;
    // float R = (147/2)*((V_r + V_l)/(V_r - V_l));
    // float W = (V_r - V_l)/147;

    // if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    // {
    //     //assignment
    //     if (target_speed_left == target_speed_right)
    //     {
    //         theta = theta;
    //         x = x + V * cos(theta) * 50/1000;
    //         y = y + V * sin(theta) * 50/1000;
    //         PrintPose();
    //     } else
    //     {   theta = theta;
    //         float theta_prime = theta + W*50/1000; 
    //         x = x - R*sin(theta) + R*sin(theta_prime);
    //         y = y + R*cos(theta) + R*cos(theta_prime);
    //         PrintPose();
    //     }
        
        
    // }

    // time_now = millis();
    // float v= 0.5*(target_speed_left+target_speed_right);
    // float w= (target_speed_right - target_speed_left)/147; 
    // float R= 0.5*147*((target_speed_right+target_speed_left)/(target_speed_right-target_speed_left));
    // float t = 50*0.001;
    // if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    // {    
    //     if(target_speed_left != target_speed_right){  //curved and point-turn movement  
    //         x= x-R*sin(theta) + R*sin(theta+w*t);      //x=x-Rsin(theta) +Rsin(theta+ w*delta t)
    //         y= y+R*cos(theta) - R*cos(theta +w*t);      //y=y+Rcos(theta) -Rcos(theta +w*delta t)
    //         theta= theta + w* t;  //theta +w*delta t
    //         PrintPose();             
    //     }else   //straight
    //     {
    //         x= x + v*cos(theta)*t;     // x+Vcos(theta) * delta t
    //         y= y+v*sin(theta) *t;      //y+Vsin(theta)* delta t
    //         theta= 0;
    //         PrintPose();
    //     }   
    //     time_prev=time_now;
        
    // }

     time_now = millis();
    float V_r = RomiEncoders.ReadVelocityRight();
    float V_l = RomiEncoders.ReadVelocityLeft();
    float v= 0.5*(V_l+V_r);
    float w= (V_r - V_l)/147; 
    float R= 0.5*147*((V_r+V_l)/(V_r-V_l));
    float t = 50*0.001;
    if(time_now - time_prev >= 50) //update every 50ms for practical reasons
    {    
        if(target_speed_left != target_speed_right){  //curved and point-turn movement  
            x= x-R*sin(theta) + R*sin(theta+w*t);      //x=x-Rsin(theta) +Rsin(theta+ w*delta t)
            y= y+R*cos(theta) - R*cos(theta +w*t);      //y=y+Rcos(theta) -Rcos(theta +w*delta t)
            theta= theta + w* t;  //theta +w*delta t
            PrintPose();             
        }else   //straight
        {
            x= x + v*cos(theta)*t;     // x+Vcos(theta) * delta t
            y= y+v*sin(theta) *t;      //y+Vsin(theta)* delta t
            theta= 0;
            PrintPose();
        }   
        time_prev=time_now;
        
    }

    
    
}

