#include <Arduino.h>
#include <Rangefinder.h>
#include <Romi32U4.h>

Rangefinder rangefinder;
long duration;

void Rangefinder::setup()
{
    pinMode(triggerPin, OUTPUT);
    pinMode(echoPin, INPUT);
    Serial.begin(9600);
}

void Rangefinder::loop()
{
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    delay(100);
}

float Rangefinder::getDistanceCM()
{

    return duration * 0.034 / 2;
}