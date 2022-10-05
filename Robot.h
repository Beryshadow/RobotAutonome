/*
-----------------------
|                     |
|      court          |
|                     |
|                     |
-----------------------

3657.6mm long by
914.4mm high
*/

#ifndef Robot
#define Robot
#include <Arduino.h>
#include <DFRobot_ICG20660L.h>
#include <Pixy2.h>

class Stepper
{
private:
    int stepPin;
    int dirPin;
    int currentLoc;
    bool reversed;

public:
    Stepper(int stepPin, int dirPin, bool reversed);
    void setup();            // TODO
    void step(int steps);    // TODO
    void dirPin(bool dir);   // TODO
    void stepPin(bool step); // TODO
    int getLoc();            // TODO
};

class Ball
{
private:
    int x;
    int y;
    int xSize;
    int ySize;
    bool isReachable;

public:
    Ball(int x, int y, int xSize, int ySize, bool isReachable); // TODO
    int getX();                                                 // TODO
    int getY();                                                 // TODO
    int getXSize();                                             // TODO
    int getYSize();                                             // TODO
    bool getIsReachable();                                      // TODO
    bool setIsReachable(bool isReachable);                      // TODO
}

class LimitSwitch
{
private:
    int pin;
    bool reversed;

public:
    LimitSwitch(int pin, bool reversed); // TODO
    bool isPressed();                    // TODO
    void setup();                        // TODO
}

class Robot
{
private:
    Stepper roue1;
    Stepper roue2;
    Stepper roue3;
    Stepper roue4;
    Stepper palette;               // hockey palette
    Stepper trapDoor;              // trap door
    DFRobot_ICG20660L_IIC gyro;    // gyro
    Pixy2 pixy;                    // camera
    LimitSwitch limitSwitchBottom; // limit switch du bas
    LimitSwitch limitSwitchLeft;   // limit switch du coté gauche
    Ball balls[40];                // array of estimated ball locations
    bool calibratedGyro;           // true if the gyro is calibrated
    bool calibratedLoc;            // true if the location is calibrated
    int estimatedXLoc;             // en relation avec le bas gauche du court en mm
    int estimatedYLoc;             // en relation avec le bas gauche du court en mm

public:
    Robot(Stepper roue1, Stepper roue2, Stepper roue3, Stepper roue4, Stepper palette, Stepper trapDoor DFRobot_ICG20660L gyro, Pixy2 pixy, LimitSwitch limitSwitch1, LimitSwitch limitSwitch2);
    void setup();                                // setup the robot (pins gyro, camera, etc.)
    void calibrateGyro();                        // TODO calibrate the gyro
    void calibrateLocation();                    // TODO calibrate the location of the robot
    void sweep();                                // TODO sweep dumbly just to get the missed balls (might remove)
    void block();                                // TODO attempt to block the ball (might remove)
    void shoot();                                // TODO will shoot the balls gathered without moving
    void move(int x, int y);                     // TODO in relation to the bottom left corner of the court
    void turnWhileGoto(int x, int y, int angle); // TODO is gonna turn while moving to the point (usefull for fast movements)
    void pointAndGoto(int x, int y, int angle);  // TODO turns in a direction before
    void turn(int angle);                        // TODO just used for turning
    void scanForBall();                          // TODO find the closest ball in view
    void clearBalls();                           // TODO clear the array of balls
}
#endif
