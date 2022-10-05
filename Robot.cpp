#include "Robot.h"

#define I2C
//#define UART
//#define SPI_SS

#ifdef I2C

#include <Pixy2I2C.h>
Pixy2I2C pixy;

#else
#ifdef UART

#include <Pixy2UART.h>
Pixy2UART pixy;

#else
#ifdef SPI_SS

#include <Pixy2SPI_SS.h>
Pixy2SPI_SS pixy;

#else

#include <Pixy2.h>
Pixy2 pixy;

#endif
#endif
#endif

// new robot
Robot::Robot(Stepper roue1, Stepper roue2, Stepper roue3, Stepper roue4, Stepper palette, Stepper trapDoor, DFRobot_gyro20660L gyro, Pixy2 pixy, LimitSwitch limitSwitch1, LimitSwitch limitSwitch2) : roue1(roue1), roue2(roue2), roue3(roue3), roue4(roue4), ball(ball), palette(palette), Stepper(trapDoor), gyro(gyro), pixy(pixy), limitSwitch1(limitSwitch1), limitSwitch2(limitSwitch2), calibratedGyro(false), calibratedLoc(false), estimatedXLoc(0), estimatedYLoc(0), Ball ball[40]{};

Robot::setup()
{
    // setup all the steppers
    roue1.setup();
    roue2.setup();
    roue3.setup();
    roue4.setup();
    palette.setup();
    trapDoor.setup();
    // setup the limit switches
    limitSwitch1.setup();
    limitSwitch2.setup();
    // init the pixy
    pixy.init();
}

Robot::calibrateGyro()
{
    // TODO
}

Robot::calibrateLoc()
{
    // TODO
}

Robot::sweep()
{
    // TODO
}
Robot::block()
{
    // TODO
}

Robot::shoot()
{
    // TODO
}

Robot::move(int x, int y)
{
    // TODO
}

Robot::turnWhileGoto(int x, int y, int angle)
{
    // TODO
}

Robot::pointAndGoto(int x, int y, int angle)
{
    // TODO
}

Robot::turn(int angle)
{
    // TODO
}

Robot::scanForBall()
{
    // TODO
}

Robot::clearBalls()
{
    // TODO
}

//-----------------------------------------------------------------------------

Stepper::Stepper(int stepPin, int dirPin, bool reversed) : stepPin(stepPin), dirPin(dirPin), reversed(reversed), loc(0){};

Stepper::setup()
{
    // TODO
}

Stepper::step(int steps)
{
    // TODO
}

Stepper::dirPin(bool dir)
{
    // TODO
}

Stepper::stepPin(bool step)
{
    // TODO
}

Stepper::getLoc()
{
    // TODO
}

//-----------------------------------------------------------------------------
Ball::Ball(int x, int y, int xSize, int ySize, bool isReachable) : x(x), y(y), xSize(xSize), ySize(ySize), isReachable(isReachable){};

Ball::getX()
{
    // TODO
}

Ball::getY()
{
    // TODO
}

Ball::getXSize()
{
    // TODO
}

Ball::getYSize()
{
    // TODO
}

Ball::getIsReachable()
{
    // TODO
}

Ball::setIsReachable(bool isReachable)
{
    // TODO
}

//-----------------------------------------------------------------------------

LimitSwitch::LimitSwitch(int pin) : pin(pin){};

LimitSwitch::setup()
{
    // TODO
}

LimitSwitch::getPin()
{
    // TODO
}

LimitSwitch::isPressed()
{
    // TODO
}

//-----------------------------------------------------------------------------