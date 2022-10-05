#ifndef Robot
#define Robot
#include <Arduino.h>

class Stepper
{
private:
    int stepPin;
    int dirPin;
    int currentLoc;
    bool reversed;
}

#endif
