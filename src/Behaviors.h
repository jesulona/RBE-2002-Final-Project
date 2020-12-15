#ifndef BEHAVIORS
#define BEHAVIORS

#include <Romi32U4.h>

class Behaviors{
    private:
        long timeNow = 0;
        long timeEndTurn = 0;
        int threshold = 450;
        int threshold_pick_up = 1500;
        int data[3] = {0};
        int numCol = 0;
        
long startX = 0;
long startY= 0;
long startZ=0;
        enum ROBOT_STATE {IDLE, IDLEWALL1, DRIVEWALL1, DRIVEWALL2, REVERSEWALL1, TURNWALL1, UPRAMP};
        ROBOT_STATE robot_state = IDLE; //initial state: IDLE
         
    public:
        void Init(void); 
        void Stop(void);
        void Run(void);
        boolean DetectCollision(void);
        boolean DetectBeingPickedUp(void);
        boolean CloseToWall(void);
        boolean DetectCollision3(void);
        boolean DetectCollision2(void);
        boolean DetectRamp(void);
};

#endif