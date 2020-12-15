
class Rangefinder {
public:

    static const int triggerPin = 12;
    static const int echoPin = 0;

    void setup();

    void loop();
    
    float getDistanceCM();

};