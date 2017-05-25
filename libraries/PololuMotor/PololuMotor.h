/*
  Helper library for the Pololu Motors used with the 
  University of Melbourne Robotics and Automation
  Class 2016
  Nathan H Murfey
*/

// YOU NEED TO INCLUDE THE INTERRUPT SERVICE
  // FOR THE ENCODER TO WORK!!!!
  // add the attachInterrup in the setup
  // then call the tick() method in that isr

// ensure this library description is only included once
#ifndef PololuMotor_h
#define PololuMotor_h

// include types & constants of Wiring core API
//#include "WConstants.h"
#include <Arduino.h>

// library interface description
class PololuMotor
{
  // user-accessible "public" interface
  public:
    PololuMotor(int, float, int, int, int, int, int, 
                int, int, int, int, int, int, 
                float, bool);
    void setPower(int);
    void setDirection(bool);
    long int getEncoderValue(void);
    float getDegreeRelative(void);
    void returnToHome(void);
    void tick(void);
    void inputPower(int);
    void setToZero(void);
    float getArmAngle(void);
    void doATestForMeWouldYou(void);
    void controlWithTorque(float);
    float getVelocity(void);
    float getAcceleration(void);
    int getTicks(void);
    void CheckStationary(int);
    void RunMotorController(void);
    void UpdateDesiredVelocity(float);
  // library-accessible "private" interface
  private:
    float _angleOffset;
    bool _orientation;
    int _gearRatio;
    float _pullyRatio;
    int _quadrature;
    int _encPinA;
    int _encPinB;
    int _pinVcc;
    int _pinGnd;
    int _pinPwm;
    char _pinIn1;
    char _pinIn2;
    int _pinStandby;
    int _encPinVcc;
    int _encPinGnd;
    long int _ticks;
    bool _direction;
    bool _encDirection;
    int _power;
    bool _encPinALast;
    float _rads_per_tick;
    float _velocity;
    float _acceleration;
    float _desired_velocity;
    
    void setupMotorInputs(void);
    void setupMotorOutputs(void);
    void getMotorVelocity(void);
    void velocityControl(void);
};  

#endif

