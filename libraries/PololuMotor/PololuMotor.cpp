/*
 Pololumotor library used in the chess playing 
 robot for the University of Melbourne Robotics and Automation class
 Nathan Murfey
*/
// include core Wiring API
//#include "WProgram.h"

// include this library's description file
#include "PololuMotor.h"

// include description files for other libraries used (if any)

#include "HardwareSerial.h"

int old_time = 0;
int new_time = 1;
long double old_pos = 0.0;
const int MAX_POWER = 150;

PololuMotor::PololuMotor(int gearRatio, float pullyRatio, int quadrature, int pinVcc, int pinGnd, int pinPwm, \
                          int pinIn1, int pinIn2, int standBy, int encPinA, int encPinB, \
                            int encPinVcc, int encPinGnd, float angleOffset, bool orientation)
{

  // initialize this instance's variables
  // these are dummy variables the idea would be able to enter these
  // into the program
  _angleOffset = angleOffset;
  _orientation = orientation;
  _gearRatio = gearRatio;
  _pullyRatio = pullyRatio;
  _quadrature = quadrature;
  _pinVcc = pinVcc;  // driver pins
  _pinGnd = pinGnd;
  _pinPwm = pinPwm;
  _pinIn1 = pinIn1;
  _pinIn2 = pinIn2;
  _pinStandby = standBy;


  _encPinA = encPinA;   // encoder pins
  _encPinB = encPinB;
  _encPinVcc = encPinVcc;
  _encPinGnd = encPinGnd;

  _direction = true;  // default forward
  _encDirection = true;
  _ticks = 0;      // set as 0 relative 
  _rads_per_tick = 2*M_PI/(_quadrature*_gearRatio*_pullyRatio);
  // set the pins as outputs for control
  pinMode(_pinVcc, OUTPUT);
  pinMode(_pinGnd, OUTPUT);
  pinMode(_pinPwm, OUTPUT);
  pinMode(_pinIn1, OUTPUT);
  pinMode(_pinIn2, OUTPUT);
  pinMode(_pinStandby, OUTPUT);

  digitalWrite(_pinVcc, HIGH);
  digitalWrite(_pinGnd, LOW);
  digitalWrite(_pinStandby, HIGH);

  // going to have to handle the interrupts in the arduino code!!
  //attachInterrupt(digitalPinToInterrupt(_encPinA), tick, CHANGE);
  // set the pins for inputs of the encodeer 
  pinMode(_encPinA, INPUT);
  pinMode(_encPinB, INPUT);
  pinMode(_encPinVcc, OUTPUT);
  pinMode(_encPinGnd, OUTPUT);

  digitalWrite(_encPinVcc, HIGH);
  digitalWrite(_encPinGnd, LOW);

  // set the default direction to forwards
  digitalWrite(_pinIn1, true);
  digitalWrite(_pinIn2, false);

  // do whatever is required to initialize the library
  pinMode(13, OUTPUT);
  //Serial.begin(9600);
  //int time = millis();
  // set the initial power to 0
  setPower(0);
}

// Public Methods //////////////////////////////////////////////////////////////
// Functions available in Wiring sketches, this library, and other libraries

void PololuMotor::setToZero(){
  // used with the limit switch to have the relative angle
  old_time = millis();
  old_pos = getArmAngle();
  _ticks = 0;
}

int PololuMotor::getTicks(void){
  return _ticks;
}

void PololuMotor::inputPower(int input){

  int resolution = 255;

  if (input >= 0){
    //Serial.print("Forward --  ");
    setDirection(true);  
    int input = int(resolution*input);
    //Serial.print(input);
    analogWrite(_pinPwm, input);
  } else {
    setDirection(false);
    //Serial.print("Backwards - ");
    int input = int(resolution*(-1)*input);
    //Serial.print(input);
    analogWrite(_pinPwm, input);
  }

 // Serial.print("  ");
}

void PololuMotor::setPower(int input)
{
  //int resolution = 255;
  //int input = int(resolution*percentage);
  if (_orientation){
    // if positive power moves positive angle 
    if (input >= 0) {
      //Serial.print(" -!POS!- ");
      setDirection(HIGH);
      input = input;
      //_velocity = _velocity + (timeDifference)*distance
    } else if (input < 0) {
      //Serial.print(" -!NEG!- ");
      setDirection(LOW);
      input = -1*input;
    } else {
      //Serial.print(" -!SHIT- ");
    }
  } else {
    // if positive power moves negative angle 
    if (input >= 0) {
      //Serial.print(" -!POS!- ");
      setDirection(LOW);
      input = input;
    } else if (input < 0) {
      //Serial.print(" -!NEG!- ");
      setDirection(HIGH);
      input = -1*input;
    } else {
      //Serial.print(" -!SHIT- ");
    }
  }
  


  //Serial.print(" Power -- ");
  //Serial.print(input);
  analogWrite(_pinPwm, input);
}

void PololuMotor::setDirection(bool direction){
  // true is forward, false is reverse
  if(direction){
    //Serial.print(" -FOR- ");
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, LOW);
  } else {
    //Serial.print(" -BAK- ");
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, HIGH);
  }
}

long int PololuMotor::getEncoderValue(void){
  return _ticks;
}

float PololuMotor::getDegreeRelative(void){
  // return the angle defined by the ticks
  // may require an offset value  if home and zero degrees are not aligned you dig?
  //number of ticks per turn of the encoder
  float degreesInCircle = 360;
  float degreesPerTick = (degreesInCircle/(_quadrature*_gearRatio));
  float angle = degreesPerTick * _ticks;
  return -1*angle;

}

float PololuMotor::getVelocity(void){

  //long int new_pos = _ticks;
 // long int new_time = millis();
  //_velocity = (new_pos-old_pos) * 1000 /(new_time-old_time);
  //old_pos = new_pos;
  //old_time = new_time;

  return _velocity;   // i think this will be in degrees per sec
}

float PololuMotor::getAcceleration(void){
  return _acceleration;
}


float PololuMotor::getArmAngle(void){
  float degreesInCircle = 360;
  float degreesPerTick = (degreesInCircle/(_quadrature*_gearRatio));
  float angle = degreesPerTick * _ticks;
  angle =  angle;
  float armAngle = angle / _pullyRatio;
  armAngle = armAngle + _angleOffset;
  return armAngle;

}


void PololuMotor::CheckStationary(int ticks){
  if(ticks == _ticks){
    _velocity = 0;
  }
}

void PololuMotor::tick(void){
  // this needs to be called in the appropriate interrupt service routine for this motors encoders
  new_time = millis();
  int Lstate;
  Lstate =  digitalRead(_encPinA);


    if((_encPinALast == LOW) && Lstate==HIGH)  // going from HIGH to LOW then reading the value on encoderPinB
    {
      int val = digitalRead(_encPinB);
      if(val == LOW && _encDirection)
      {
        _encDirection = false; //Reverse
      }
      else if(val == HIGH && !_encDirection)
      {
        _encDirection = true;  //Forward
      }
    }

    // GETTING VELOCITY

    int new_pos = getArmAngle();
 
    if(new_time-old_time == 0){
      _velocity = _velocity;
    } else {
      _velocity = 1000*((new_pos-old_pos) /(new_time-old_time));
    }
    
    old_pos = new_pos;
    old_time = new_time;

    _encPinALast = Lstate;
    int timeStep = new_time - old_time;
    Serial.print("new time: ");
    Serial.println(new_time);
    Serial.print("old time: ");
    Serial.println(old_time);
    Serial.print("timestep: ");
    Serial.println(timeStep);
    Serial.print("millis: ");
    Serial.println(millis());


   // if(_orientation){
      if(!_encDirection){
        //positive direction
         _ticks++;
      } 
      else { 
        //negative direction
         _ticks--;
      }
      // i think i need to update the velocity in here
}
// Private Methods /////////////////////////////////////////////////////////////
// Functions only available to other functions in this library

void PololuMotor::doATestForMeWouldYou(void)
{
  for (int i = 500; i > 0; i = i - 100){
      digitalWrite(13, HIGH);
      delay(i);
      digitalWrite(13, LOW);
      delay(i);
  }

}

void PololuMotor::controlWithTorque(float torque){
  float driverMaxCurrent = 2.1;     //amps
  float encoderMaxInput = 255;
  float motorGearRatio = _gearRatio;


}

void PololuMotor::UpdateDesiredVelocity(float desired_velocity){
  _desired_velocity = desired_velocity;
}

void PololuMotor::RunMotorController(void){
  float k = 2;
  float error = (_desired_velocity - _velocity)*k;
  float input = 0;
  //
  while(_velocity != _desired_velocity){
    if (abs(error)>0.5){
      if (error > 0){
        if(input > 127){
          input = 127;
        } else {
          input = input + 1;
          setPower(input);
        }
      } else {
          if(input < -127){
            input = -127;
          } else {
            input = input - 1;
            setPower(input);
          }
        }
    } else {
        break;
    } 
    Serial.print("Desired: ");
    Serial.print(_desired_velocity);
    Serial.print(", Actual: ");
    Serial.print(_velocity);    
    Serial.print(", Input: ");
    Serial.println(input);
    error = (_desired_velocity - _velocity)*k;
  }
  //setPower(input);
}

void PololuMotor::setupMotorInputs(void){
  // TO DO 
}

void PololuMotor::setupMotorOutputs(void){
  // TO DO

}


