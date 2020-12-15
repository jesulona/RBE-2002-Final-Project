#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp = 0.5; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Ki = 0.1; 
        float E_left = 0; 
        float E_right = 0;
        int counts = 1450; //assignment

        float error_distance = 0;
        float E_distance = 0;
        float E_theta = 0;
        float prev_time = 0; 
        float prev_vel = 0 ; 
        float acceleration = 0;
        float velocity = 0; 

        float error_distance_sum = 0;
        float error_theta_sum = 0;
        float error_theta = 0; 

    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        void Init(void);
        void Run(float, float); //speed left, speed right
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean Reverse(int, int);
        void Process(float, float);
        boolean MoveToPosition(float,float); //target_x, target_y
        void Stop(void);
};

#endif