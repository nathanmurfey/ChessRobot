#include <Trajectory.h>
#include <Jacobian.h>
#include <PololuMotor.h>
#include <MatrixMath.h>
#include <Kinematics.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// in house library written for ease of communication with
// the different pololu motors for use in the UoM Robotics
// and Automation class 2016

// CHECKLIST each motor requires::
/*
   For each motor:
    input
    limit switch
    encoders ticking

   For the primitives
    need to update the theta values so that
    the motors know when to stop
*/



// -----------------------------------------------------------------------------------------------
// ------------------------------- PARAMS --------------------------------------------------------
// -----------------------------------------------------------------------------------------------
//Parameters (mm)
//Base
#define joint0X_originTo0  0     //defined origin (x,y,z) coincident on this point
#define joint0Y_originTo0  0     //defined origin (x,y,z) coincident on this point
#define joint0Z_originTo0  0     //defined origin (x,y,z) coincident on this point

//Link 1 (imaginary link, base to motor)
#define joint1X            30 //COME BACK TO THIS
#define joint1Z            100 //COME BACK TO THIS
#define joint1Angle        atan(joint1X/joint1Z) //COME BACK TO THIS
#define link1Length        sqrt(joint1X*joint1X + joint1Z*joint1Z)
#define joint1X_0to1       link1Length //x1 axis oriented along link1
#define joint1Y_0to1       0
#define joint1Z_0to1       0

//Link 2 (physical link, closer to base)
#define link2Length        270
#define joint2X_1to2       link2Length //x2 axis orented along link2
#define joint2Y_1to2       0
#define joint2Z_1to2       0

//Link 3 (physical link, closer to end effector)
#define link3Length        300
#define joint3X_2to3       link3Length //x3 axis oriented along linkk3
#define joint3Y_2to3       0
#define joint3Z_2to3       0

//Forward Kinematics
#define N           4 //transformation matrix size

//Chessboard (assumes inertial frame oriented with Y along board's centreline)
#define offsetY            260 //distance base is back from the board's edge
#define offsetZ            100 //height of 'floor' we consider for kinematics

double chessX[8] = { -105, -75, -45, -15, 15, 45, 75, 105}; //columns
double chessY[8] = {15 + offsetY, 45 + offsetY, 75 + offsetY, 105 + offsetY, 135 + offsetY,
                    175 + offsetY, 205 + offsetY, 235 + offsetY
                   };  //rows
double tolerance = 15; //half size of chess square (mm)

void JointToTask(double theta1d, double theta2d, double theta3d, double xyCoords[2]);
void taskToJoint(double xValue, double yValue, double thetaVectorD[3]);
int getColumn(double xInput);
int getRow(double yInput);
void matrixMutliply(double inputM1[4][4], double inputM2[4][4], double outM[4][4]);

double xyCoords[2];
double thetaVectorD[3];

// ---------------------------- GRIPPER

const byte gripperGround = 27;
const byte gripperOpenSwitch = 29;
const byte gripperClosedSwitch = 31;
const byte gripperVcc = 35;
const byte gripperGnd = 37;
const byte gripperPWM = 3;
const byte gripperIn1 = A5; 
const byte gripperIn2 = A6;
const byte gripperStandby = A14;


// -----------------------------------------------------------------------------------------------
// ------------------------------- MOTOR PINS ----------------------------------------------------
// -----------------------------------------------------------------------------------------------

const byte motorOneLimitSwitchPin = A0;
const byte motorOneInterruptPin = 21;
const byte motorOneGearRatio = 34;
const float motorOnePullyRatio = 3;
const byte motorOneQuadrature = 24;
const byte motorOneVccPin = A6;
const byte motorOneGndPin = A7;
const byte motorOnePwmPin = 4;
const byte motorOneIn1 = 53;
const byte motorOneIn2 = 43;
const byte motorOneStandby = A10;
const byte motorOneEncPinA = 21;
const byte motorOneEncPinB = 17;
const byte motorOneEncVcc = 23;
const byte motorOneEncGnd = 25;

const byte motorTwoLimitSwitchPin = A1;
const byte motorTwoInterruptPin = 20;
const byte motorTwoGearRatio = 131;
const float motorTwoPullyRatio = 60.0/36;
const byte motorTwoQuadrature = 30;
const byte motorTwoVccPin = A6;
const byte motorTwoGndPin = A7;
const byte motorTwoPwmPin = 5;
const byte motorTwoIn1 = 51;
const byte motorTwoIn2 = 41;
const byte motorTwoStandby = A11;
const byte motorTwoEncPinA = 20;
const byte motorTwoEncPinB = 16;
const byte motorTwoEncVcc = 23;
const byte motorTwoEncGnd = 25;

const byte motorThreeLimitSwitchPin = A2;
const byte motorThreeInterruptPin = 19;
const byte motorThreeGearRatio = 75;
const float motorThreePullyRatio = 3;
const byte motorThreeQuadrature = 24;
const byte motorThreeVccPin = A6;
const byte motorThreeGndPin = A7;
const byte motorThreePwmPin = 6;
const byte motorThreeIn1 = 49;
const byte motorThreeIn2 = 39;
const byte motorThreeStandby = A12;
const byte motorThreeEncPinA = 19;
const byte motorThreeEncPinB = 15;
const byte motorThreeEncVcc = 23;
const byte motorThreeEncGnd = 25;

const byte motorFourLimitSwitchPin = A3;
const byte motorFourInterruptPin = 18;
const byte motorFourGearRatio = 34;
const float motorFourPullyRatio = 3;
const byte motorFourQuadrature = 24;
const byte motorFourVccPin = A6;
const byte motorFourGndPin = A7;
const byte motorFourPwmPin = 7;
const byte motorFourIn1 = 47;
const byte motorFourIn2 = 37;
const byte motorFourStandby = A13;
const byte motorFourEncPinA = 18;
const byte motorFourEncPinB = 14;
const byte motorFourEncVcc = 23;
const byte motorFourEncGnd = 25;


//const byte motorOneLimitSwitchPin = A4;
//const byte motorFiveInterruptPin = 21;
//const byte motorFiveGearRatio = 34;
//const byte motorFiveQuadrature = 24;
//const byte motorFiveVccPin = 24;
//const byte motorFiveGndPin = 25.;
//const byte motorFivePwmPin = 2;
//const byte motorFiveIn1 = A8;
//const byte motorFiveIn2 = A9;
//const byte motorFiveStandby = 8;
//const byte motorFiveEncPinA = 21;
//const byte motorFiveEncPinB = 17;
//const byte motorFiveEncVcc = 22;
//const byte motorFiveEncGnd = 23;

// -----------------------------------------------------------------------------------------------
// ------------------------------- MOTOR INIT ----------------------------------------------------
// -----------------------------------------------------------------------------------------------

PololuMotor motorOne(motorOneGearRatio, motorOnePullyRatio, motorOneQuadrature, motorOneVccPin, motorOneGndPin,
                     motorOnePwmPin, motorOneIn1, motorOneIn2, motorOneStandby, motorOneEncPinA,
                     motorOneEncPinB, motorOneEncVcc, motorOneEncGnd, (102.5-1.444), true);
PololuMotor motorTwo(motorTwoGearRatio, motorTwoPullyRatio, motorTwoQuadrature, motorTwoVccPin, motorTwoGndPin,
                     motorTwoPwmPin, motorTwoIn1, motorTwoIn2, motorTwoStandby, motorTwoEncPinA,
                     motorTwoEncPinB, motorTwoEncVcc, motorTwoEncGnd, (120-7.79), true);
PololuMotor motorThree(motorThreeGearRatio, motorThreePullyRatio, motorThreeQuadrature, motorThreeVccPin, motorThreeGndPin,
                       motorThreePwmPin, motorThreeIn1, motorThreeIn2, motorThreeStandby, motorThreeEncPinA,
                       motorThreeEncPinB, motorThreeEncVcc, motorThreeEncGnd, (231+2), false);
PololuMotor motorFour(motorFourGearRatio, motorFourPullyRatio, motorFourQuadrature, motorFourVccPin, motorFourGndPin,
                       motorFourPwmPin, motorFourIn1, motorFourIn2, motorFourStandby, motorFourEncPinA,
                       motorFourEncPinB, motorFourEncVcc, motorFourEncGnd, 380, false);


//PololuMotor motorFour(motorFourGearRatio, motorFourQuadrature, motorFourVccPin, motorFourGndPin,
//motorFourPwmPin, motorFourIn1, motorFourIn2, motorFourStandby, motorFourEncPinA,
//motorFourEncPinB, motorFourEncVcc, motorFourEncGnd,true);
//PololuMotor motorFive(motorFiveGearRatio, motorFiveQuadrature, motorFiveVccPin, motorFiveGndPin,
//motorFivePwmPin, motorFiveIn1, motorFiveIn2, motorFiveStandby, motorFiveEncPinA,
//motorFiveEncPinB, motorFiveEncVcc, motorFiveEncGnd, true);

String command;
 float Q_ik[3];
Kinematics kinematics;
Jacobian jacobian;
Trajectory trajectory;

long int delta_t_start = 0;
long int delta_t_end = 0;
float delta_t;
float Q_o[4];
float dot_Q_o[4];

const int JAC_INV_ROWS = 4;
const int JAC_INC_COLS = 3;

float J_inverse[JAC_INV_ROWS][JAC_INC_COLS];
float J_noninverted[JAC_INC_COLS][JAC_INV_ROWS];
// -----------------------------------------------------------------------------------------------
// ------------------------------- INTERRUPT SERVICE ROUTINES ------------------------------------
// -----------------------------------------------------------------------------------------------

void ISR_motorOne(void) {
  //noInterrupts();
  motorOne.tick();
  //interrupts();
}

void ISR_motorTwo(void) {
 // noInterrupts();
  motorTwo.tick();
 // interrupts();
}

void ISR_motorThree(void) {
 // noInterrupts();
  motorThree.tick();
 // interrupts();
}

void ISR_motorFour(void){
 // noInterrupts();
  motorFour.tick();
 // interrupts();
}
/*
  void ISR_motorFive(void){
  motorFive.tick();
  }

*/
bool am_moving = false;
float xdot_d;
float ydot_d;
float zdot_d;
float x_d;
float y_d;
float z_d;
// -----------------------------------------------------------------------------------------------
// ------------------------------- SETUP AND LOOP ------------------------------------------------
// -----------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(gripperGnd, OUTPUT);
  pinMode(gripperVcc, OUTPUT);
  pinMode(gripperGround, OUTPUT);
  pinMode(gripperStandby, OUTPUT);
  pinMode(gripperIn1, OUTPUT);
  pinMode(gripperIn2, OUTPUT);

  pinMode(gripperPWM, OUTPUT);
  // -- FOR
  digitalWrite(gripperIn1,HIGH);
  digitalWrite(gripperIn2,LOW);

  // -- BACK
  digitalWrite(gripperIn1,LOW);
  digitalWrite(gripperIn2,HIGH);
  
  digitalWrite(gripperStandby, HIGH);
  digitalWrite(gripperGround, HIGH);
  digitalWrite(gripperGnd, LOW);
  digitalWrite(gripperVcc, HIGH);
  digitalWrite(gripperGround, LOW);
  
  pinMode(gripperClosedSwitch, INPUT_PULLUP);
  pinMode(gripperOpenSwitch, INPUT_PULLUP);
  
  //PrintWelcomeMessage();
  SetupMotors();
  SetupLimitSwitches();
  ResetMotorZeros();
  SendReadyToArduino();
  //FaceForward();
  //HackMotorMove();
  delta_t_end =millis();

  Q_ik[0] = motorOne.getArmAngle();
  Q_ik[1] = motorTwo.getArmAngle();
  Q_ik[2] = motorThree.getArmAngle();
}

void loop() {

  Serial.println("GRIP!");

  while(digitalRead(gripperOpenSwitch)){

    digitalWrite(gripperIn1,HIGH);
    digitalWrite(gripperIn2,LOW);
    analogWrite(gripperPWM, 255);
  }

  Serial.println("GRIPPER OPEN");
  analogWrite(gripperPWM, 0);
  while(digitalRead(gripperClosedSwitch)){
    
    digitalWrite(gripperIn1,LOW);
    digitalWrite(gripperIn2,HIGH);
    analogWrite(gripperPWM, 255);
  }
  Serial.println("GRIPPER CLOSED");
  //delay(1000);
/*
  float Q_e[4];
  if(Serial.available()){
    // start the task
    kinematics.Joint2TaskUpdate(Q_e);

    float xdot = 0;
    float ydot = 0;
    float zdot = 0;

    float initConditions[6];
    initConditions[0] = kinematics.Joint2TaskX();
    initConditions[1] = kinematics.Joint2TaskY();
    initConditions[2] = kinematics.Joint2TaskZ();
    initConditions[3] = 0;
    initConditions[4] = 0;
    initConditions[5] = 0;

    float finalConditions[6];
    finalConditions[0] = 580;
    finalConditions[1] = 10;
    finalConditions[2] = kinematics.Joint2TaskZ();
    finalConditions[3] = 0;
    finalConditions[4] = 0;
    finalConditions[5] = 0;

    trajectory.TrajectoryBuild(initConditions, finalConditions, 10);
    command = Serial.readString();
    am_moving  = true;
    Serial.println("TRAJ");
    }
  // ---------------------------- FORWARD KINEMATICS
  // ---------------------------- Position
  Q_e[0] = motorOne.getArmAngle();
  Q_e[1] = motorTwo.getArmAngle();
  Q_e[2] = motorThree.getArmAngle();
  Q_e[3] = motorFour.getArmAngle();
  kinematics.Joint2TaskUpdate(Q_e);
  float x_w = kinematics.Joint2TaskX();
  float y_w = kinematics.Joint2TaskY();
  float z_w = kinematics.Joint2TaskZ();

  // ---------------------------- Velocity
  delta_t_start = millis();
  delta_t = delta_t_end - delta_t_start;
  dot_Q_o[0] = (Q_e[0]- Q_o[0])/delta_t;
  dot_Q_o[1] = (Q_e[1]- Q_o[1])/delta_t;
  dot_Q_o[2] = (Q_e[2]- Q_o[2])/delta_t;
  dot_Q_o[3] = (Q_e[3]- Q_o[3])/delta_t;
  delta_t_end = delta_t_start;
  Q_o[0] = Q_e[0];
  Q_o[1] = Q_e[1];
  Q_o[2] = Q_e[2];
  Q_o[3] = Q_e[3];
  // ---------------------------- INVERSE KINEMATICS
  
  // ---------------------------- TRAJECTORY GENERATION
  
    trajectory.TrajectoryUpdate();
    float xdot_d = trajectory.getVelocityX();
    float ydot_d = trajectory.getVelocityY();
    float zdot_d = trajectory.getVelocityZ();
    float x_d = trajectory.getPositionX();
    float y_d = trajectory.getPositionY();
    float z_d = trajectory.getPositionZ();
    
 //Q_ik[0] = kinematics.Task2JointQ1(x_d, y_d);
  //Q_ik[1] = kinematics.Task2JointQ2(x_d, y_d);
  //Q_ik[2] = kinematics.Task2JointQ3(x_d, y_d);

  Q_ik[0] = kinematics.Task2JointQ1(x_w, y_w);
  Q_ik[1] = kinematics.Task2JointQ2(x_w, y_w);
  Q_ik[2] = kinematics.Task2JointQ3(x_w, y_w);

  jacobian.UpdateInverseJacobian(Q_e);
  jacobian.ReturnInverseJacobian((float*)J_inverse);

  float Q1Dot_d = (J_inverse[0][0]*xdot_d+J_inverse[0][1]*ydot_d+J_inverse[0][2]*zdot_d)*(180/PI);
  float Q2Dot_d = (J_inverse[1][0]*xdot_d+J_inverse[1][1]*ydot_d+J_inverse[1][2]*zdot_d)*(180/PI);
  float Q3Dot_d = (J_inverse[2][0]*xdot_d+J_inverse[2][1]*ydot_d+J_inverse[2][2]*zdot_d)*(180/PI);
  float Q4Dot_d = (J_inverse[3][0]*xdot_d+J_inverse[3][1]*ydot_d+J_inverse[3][2]*zdot_d)*(180/PI);  
  // ---------------------------- LINEAR CONTROLLER
  // --------------- ERRORS
  // PROPORTIONAL ERROR
  float Q1Dot_e =  Q1Dot_d - dot_Q_o[0];
  float Q2Dot_e =  Q2Dot_d - dot_Q_o[1];
  float Q3Dot_e =  Q3Dot_d - dot_Q_o[2];
  // INTEGRAL ERROR
  float Q1_e =  Q_ik[0] - Q_e[0];
  float Q2_e =  Q_ik[1] - Q_e[1];
  float Q3_e =  Q_ik[2] - Q_e[2];


  //Serial.print(" [Q1]: ");
  //Serial.print(x_w);
  //Serial.print(" [Q1]: ");
  //Serial.print(y_w);
  //Serial.print(" [Q1]: ");
  //Serial.print(z_w);

  Serial.print("  Inverse kinematics: ");
  Serial.print(" [Q1]: ");
  Serial.print(Q_e[0]);
  Serial.print("; [Q2]: ");
  Serial.print(Q_e[1]);
  Serial.print("; [Q3]: ");
  Serial.print(Q_e[2]);
  Serial.print(" [Q1_ik]: ");
  Serial.print(Q_ik[0]);
  Serial.print("; [Q2_ik]: ");
  Serial.print(Q_ik[1]);
  Serial.print("; [Q3_ik]: ");
  Serial.print(Q_ik[2]);
  Serial.print(" Error: ");
  Serial.print(" [Q1]: ");
  Serial.print(Q1_e);
  Serial.print("; [Q2]: ");
  Serial.print(Q2_e);
  Serial.print("; [Q3]: ");
  Serial.print(Q3_e);
  */


  
  //Serial.print(";    [Q1d]: ");
  //Serial.print(Q1Dot_d); 
  //Serial.print("; [Q2d]: ");
  //Serial.print(Q2Dot_d);
  //Serial.print("; [Q3d]: ");
  //Serial.print(Q3Dot_d);
  Serial.println("");
  
  //motorOne.RunMotorController(1, Q_e[0], Q_ik[0], Q1Dot_d, dot_Q_o[1]);
  //motorTwo.RunMotorController(2, Q_e[1], Q_ik[1], Q2Dot_d, dot_Q_o[2]);
  //motorThree.RunMotorController(3, Q_e[2], Q_ik[2], Q3Dot_d, dot_Q_o[3]);

}

void RunTheCommand(void) {

  //char charBuf[50];
  //stringOne.toCharArray(charBuf, 50)

  char charBuff[12];


  if (command.substring(6, 9).equals("RST")) {
    ResetMotorZeros();
  } else if (command.substring(0, 3).equals("CMD")) {
    UpdateTaskToJointSpace( command.substring(4, 5).toInt(),
                            command.substring(5, 6).toInt());
  } else if (command.substring(6, 9).equals("RUN")) {
    MoveAllMotorsToJointSpace();
  } else if (command.substring(6, 9).equals("TST")) {
    Serial.println("[INFO] TEST SUCCESSFUL");
  } else if (command.substring(6, 7).equals("!")) {
    Serial.println("[INFO] COMMAND RECIEVED");
    Serial.println(command);
    command.toCharArray(charBuff, 12);
    //Serial.println("----");
    //Serial.println(charBuff[8]);
    //Serial.println(charBuff[9]);
    char letter = charBuff[8];
    char number = charBuff[9];
    UpdateTaskToJointSpace(int(letter), int(number));
    MoveAllMotorsToJointSpace();

  } else {
    Serial.print("[ERRO] Command Failure, header incorrect - ");
    Serial.println("check incoming instruction from pyserial");
  }

  Serial.println("[FCOM]");
  Serial.println("-----------------------");
}

String SendRequest(void) {
  Serial.println("[RQST] INSTRUCTIONS");
  delay(1000);
  while (true) {
    if (Serial.available()) {
      String message = Serial.readString();
      //Serial.println(message);
      if (message.substring(0, 6).equals("[COMM]")) {
        return message;
      }
    }
  }
}

void SendReadyToArduino(void) {
  //Serial.println("[REDY] Let's get going! - ARDUINO");
}


void SetupMotors(void) {

  attachInterrupt(digitalPinToInterrupt(motorOneInterruptPin), ISR_motorOne, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorTwoInterruptPin), ISR_motorTwo, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorThreeInterruptPin), ISR_motorThree, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motorFourInterruptPin), ISR_motorFour, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(motoFiveInterruptPin), ISR_motorFive, CHANGE);
}

// -----------------------------------------------------------------------------------------------
// ------------------------------ PRIMITIVES -----------------------------------------------------
// -----------------------------------------------------------------------------------------------

void MoveAllMotorsToJointSpace() {
  Serial.println("[INFO] Moving All Motors to new positions");
  // M1 - M3 - M2 , 120 - 60 - 70
  MoveMotorInJointSpace(motorOne, 1, 100);
  MoveMotorInJointSpace(motorThree, 3, 60);
  MoveMotorInJointSpace(motorTwo, 2, 100);
}

void ResetMotorZeros(void) {
  // Each one of the motors moves slowly back until the limit switches are toggled
  // when the limit switch goes off the motor encoder 'zero' is updated to that value.
  //Serial.println("[WARN] Clear Robot Calibrating");
  //Serial.println("[UPDT] Calibrating Motor Two");
  while (digitalRead(motorTwoLimitSwitchPin)) {
    motorTwo.setPower(100);
  }
  motorTwo.setPower(0);
  motorTwo.setToZero();

  //Serial.println("[UPDT] Calibrating Motor Three");
  while (!digitalRead(motorThreeLimitSwitchPin)) {
    motorThree.setPower(-40);
  }
  motorThree.setPower(0);
  motorThree.setToZero();

  //Serial.println("[UPDT] Calibrating Motor One");
  while (digitalRead(motorOneLimitSwitchPin)) {
    motorOne.setPower(130);
  }
  motorOne.setPower(0);
  motorOne.setToZero();

  //Serial.println("[UPDT] Calibrating Motor Four");
  while (digitalRead(motorFourLimitSwitchPin)) {
    motorFour.setPower(60);
  }
  motorFour.setPower(0);
  motorFour.setToZero();

  //Serial.println("[INFO] Calibraion Completed");
}

void HackMotorMove(void) {
  // Each one of the motors moves slowly back until the limit switches are toggled
  // when the limit switch goes off the motor encoder 'zero' is updated to that value.
  //Serial.println("[WARN] Clear Robot Calibrating");
  //Serial.println("[UPDT] Calibrating Motor Two");
  while (motorTwo.getArmAngle() > 90 ) {
    motorTwo.setPower(-150);
  }
  motorTwo.setPower(0);
 

  //Serial.println("[UPDT] Calibrating Motor Three");
  while (motorThree.getArmAngle() < 260) {
    motorThree.setPower(60);
  }
  motorThree.setPower(0);
  

  //Serial.println("[UPDT] Calibrating Motor One");
  while (motorThree.getArmAngle() > 0) {
    motorOne.setPower(-100);
  }
  motorOne.setPower(0);
 
/*
  //Serial.println("[UPDT] Calibrating Motor Four");
  while (digitalRead(motorFourLimitSwitchPin)) {
    motorFour.setPower(60);
  }
  motorFour.setPower(0);
  motorFour.setToZero();
*/
  //Serial.println("[INFO] Calibraion Completed");
}


void FaceForward(){
  while (motorOne.getArmAngle() > 0) {
    motorOne.setPower(-80);
  }
  motorOne.setPower(0);
  }

void MoveMotorInJointSpace(PololuMotor& motor, int motorNumber, int motorPower) {
  // after updating the thetavector in the task space function
  // this function will move the motors until the encoders are correct
  // this will be removec and replaced with better motion control.
  int index = motorNumber - 1;
  Serial.print("[UPDT] Moving Motor ");
  Serial.println(motorNumber);

  while (motor.getArmAngle() > thetaVectorD[index]) {
    Serial.print("[UPDT] Moving Motor ");
    Serial.print(motorNumber);
    Serial.print(" DES: ");
    Serial.print(thetaVectorD[index]);
    Serial.print(" ACC: ");
    Serial.println(motor.getArmAngle());
    Serial.print(" [X,Y, Z]: ");
    Serial.print(xyCoords[0]);
    Serial.print(" , ");
    Serial.print(xyCoords[1]);
    JointToTask(motorOne.getArmAngle(), motorTwo.getArmAngle(), motorThree.getArmAngle(), xyCoords);
    motor.setPower(-motorPower);
  }
  while (motor.getArmAngle() < thetaVectorD[index]) {
    Serial.print("[UPDT] Moving Motor ");
    Serial.print(motorNumber);
    Serial.print(" DES: ");
    Serial.print(thetaVectorD[index]);
    Serial.print(" ACC: ");
    Serial.println(motor.getArmAngle());
    JointToTask(motorOne.getArmAngle(), motorTwo.getArmAngle(), motorThree.getArmAngle(), xyCoords);
    motor.setPower(motorPower);
  }
  motor.setPower(0);

  Serial.print("[INFO] Finished Moving Motor ");
  Serial.println(motorNumber);

}


// -----------------------------------------------------------------------------------------------
// ------------------------------ KINEMATICS -----------------------------------------------------
// -----------------------------------------------------------------------------------------------

void UpdateTaskToJointSpace(int letterIndex, int numberIndex) {
  letterIndex = letterIndex - 65;   // offset the char to int
  numberIndex = numberIndex - 49;
  int xCoord = chessX[letterIndex];
  int yCoord = chessY[numberIndex];
  taskToJoint(xCoord, yCoord, thetaVectorD);
  Serial.print("[UPDT] Updated Joint Space X:[");
  Serial.print(xCoord);
  Serial.print(", ");
  Serial.print(yCoord);
  Serial.print("]");
  Serial.print(" , Q:[");
  Serial.print(thetaVectorD[0]);
  Serial.print(", ");
  Serial.print(thetaVectorD[1]);
  Serial.print(", ");
  Serial.print(thetaVectorD[2]);
  Serial.println("]");
}

//input x value, get chessboard column (A - H)
int getColumn(double xInput) {
  int i;
  char colValue;
  for (i = 1; i < 9; i++)
  {
    double difference = abs(xInput - chessX[i - 1]);
    if (difference < tolerance) {
      colValue = i + 64;
      return colValue;
    }
  }

  colValue = 'X';
  return colValue; //indicates problem
}

//input y information, get chessboard row (1 - 8)
int getRow(double yInput) {
  int j;
  for (j = 1; j < 9; j++)
  {
    if (abs(yInput - (chessY[j - 1])) < tolerance)
      return j;
  }

  return 0; //indicates problem
}

//Forward Kinematics
//given (theta1, theta2, theta3), returns (x,y)
void JointToTask(double theta1d, double theta2d, double theta3d, double xyCoords[2]) {

  double theta1 = M_PI * theta1d / 180;
  double theta2 = M_PI * theta2d / 180;
  double theta3 = M_PI * theta3d / 180;

  //Transformation matrices
  double T01[4][4] = {
    {sin(theta1), -cos(theta1), 0, 0},
    {cos(theta1), sin(theta1),  0, 0},
    {0,       0,      1, 100},
    {0,       0,      0, 1}
  };
  double T12[4][4] = {
    {cos(theta2), -sin(theta2), 0, 30},
    {0,       0,      -1, 0},
    {sin(theta2), cos(theta2),  0, 0},
    {0,       0,      0, 1}
  };
  double T23[4][4] = {
    {cos(theta3), -sin(theta3), 0, 270},
    {sin(theta3), cos(theta3),  0, 0},
    {0,       0,      1, 0},
    {0,       0,      0, 1}
  };
  double T34[4][4] = {
    {1, 0, 0, 300},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };

  double T02[4][4];
  double T03[4][4];
  double T04[4][4];

  matrixMutliply(T01, T12, T02);
  matrixMutliply(T02, T23, T03);
  matrixMutliply(T03, T34, T04);

  xyCoords[0] = -1 * T04[0][3];
  xyCoords[1] = T04[1][3];

  //printf("(x,y) are (%f,%f)\n", T04[0][3],T04[1][3]);
  return;
}


void taskToJoint(double xValue, double yValue, double thetaVectorD[3]) {
  //Inverse Kinematics
  //given (x,y), returns (theta1, theta2, theta3)
  xValue = -1 * xValue;

  //Theta1
  thetaVectorD[0] = (180 / M_PI) * (atan(xValue / yValue));

  //Theta2
  double a = link3Length;
  //double b = sqrt((xInput - joint1X*sin(theta1)*sin(theta1)) + (yInput - joint1X*cos(theta1)*cos(theta1)) + (offsetZ - joint1Z)*(offsetZ - joint1Z));
  //line above not working. this is simplification:
  double b = sqrt(xValue * xValue + yValue * yValue) - joint1X;
  double c = link2Length;

  thetaVectorD[1] = (180 / M_PI) * (acos((b * b + c * c - a * a) / (2 * b * c)));

  //Theta3
  double alpha = acos((a * a + c * c - b * b) / (2 * a * c));
  thetaVectorD[2] = (180 / M_PI) * (alpha + M_PI);

  return;
}


void matrixMutliply(double inputM1[4][4], double inputM2[4][4], double outM[4][4]) {
  //Matrix multiplication
  double sum = 0;
  int i, j, k;
  for (i = 0; i < N; i++) {
    for (j = 0; j < N; j++) {
      for (k = 0; k < N; k++) {
        sum += inputM1[i][k] * inputM2[k][j];
      }
      outM[i][j] = sum;
      sum = 0;
      //printf("%0.3f ",outM[i][j]);
    }
    //printf("\n");
  }
  return;
}

void SetupLimitSwitches() {
  pinMode(motorOneLimitSwitchPin, INPUT_PULLUP);
  pinMode(motorTwoLimitSwitchPin, INPUT_PULLUP);
  pinMode(motorThreeLimitSwitchPin, INPUT_PULLUP);
  pinMode(motorFourLimitSwitchPin, INPUT_PULLUP);
  //pinMode(motorFiveLimitSwitchPin, INPUT_PULLUP);
}
void PrintWelcomeMessage(void) {

  Serial.println("    ???+                                                                        ");
  Serial.println("    I??+                                                                        ");
  Serial.println("     ??                                                                         ");
  Serial.println("  I?++++?+         +?                                                           ");
  Serial.println("   I?++++         .DN                                                           ");
  Serial.println("   I?+++=        DNNDDD                                                         ");
  Serial.println("   I?+++=        $NNDN?           ,                                             ");
  Serial.println("   I???I?        8NDONZ         +++I        ,                                   ");
  Serial.println("    7?++         8NNNN8        ?+==++      DD?D?DZ       +==~:==                ");
  Serial.println("    I+=+          NNDD         ?=~?~=      D8OD+DDDD     ++$::=+         ?      ");
  Serial.println("    I+++          DND8         I+===+     DO888IDD8D     I?+==++       ZNNDNI   ");
  Serial.println("    I+++          NDD8          ?==+      8DDD8N          ?+~==        DNDDND   ");
  Serial.println("    I+=+          NN8D         ++===+      DZDND8D        ?=~=+         NNND    ");
  Serial.println("    ?+=+          NDOD          I==+       DDNNDD8DZ      ++===        ONNNN7   ");
  Serial.println("    ?++?         ONNON?         ?==+       ZDDDODODD      ?+~==+        DNN     ");
  Serial.println("   I?+=++        DNNODD         ?==+        8DO7?+DD      ?+~=++        DNN     ");
  Serial.println("  ?????++?      INND8ND7       ?+==+        8DD8DDN      +I?++?I        NNNN    ");
  Serial.println(" ?I?+?????I    ZDN8NNNNDZ     ??+=~+++     $DDD$DDD+    =?++==++?      NNDNNN   ");
  Serial.println("I???+++++??   DNNNNDO8NND    I?+=+==++?    DDDD8DDND    ?+++++++??   ZNNONNNNN  ");
  Serial.println("II77IIIIII?I  DNNNNNNNNNN8   ??????+???   DDNNDNNNNN8   ??III????+   8NNNNNNNNO ");
  Serial.println("--------------------------------------------------------------------------------");
  Serial.println("---------------------------------LET'S CHESS------------------------------------");
  Serial.println("--------------------------------------------------------------------------------");
}

void PrintSerialGeneralInfo(void) {
  Serial.print("[INFO] ");
  Serial.print("Q1 - ");
  Serial.print(motorOne.getArmAngle());
  Serial.print(" Q2 - ");
  Serial.print(motorTwo.getArmAngle());
  Serial.print(" Q3 - ");
  Serial.print(motorThree.getArmAngle());
  Serial.print("(x,y) = ");
  Serial.print(xyCoords[0]);
  Serial.print(",");
  Serial.print(xyCoords[1]);
  Serial.print("---");
  char chessColumn = getColumn(xyCoords[0]);
  Serial.print(chessColumn);
  Serial.print(" ");
  Serial.println(getRow(xyCoords[1]));
}
