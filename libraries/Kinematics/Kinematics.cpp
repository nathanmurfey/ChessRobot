#include "Kinematics.h"
#include "HardwareSerial.h"
#include <MatrixMath.h>
/*// -----------------------------------------------------------------------------------------------
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

double chessX[8] = {-105, -75, -45, -15, 15, 45, 75, 105}; //columns
double chessY[8] = {15+offsetY, 45+offsetY, 75+offsetY, 105+offsetY, 135+offsetY, 
                    175+offsetY, 205+offsetY, 235+offsetY};  //rows
double tolerance = 15; //half size of chess square (mm)

double xyCoords[2];
double thetaVectorD[3];
*/
#define a_0 = 100;
#define N           4 //transformation matrix size

const float L4 = 170.00;
const float L3 = 300.00;
const float L2 = 270.00;
const float L1Z = 77.55;
const float L1X = 32.44;

//void MatrixMutliply(double inputM1[4][4], double inputM2[4][4], double outM[4][4]);
Kinematics::Kinematics(void)
{

}

//------------------------------------------------------------------------------------
//------------------------FORWARD KINEMATICS------------------------------------------
//------------------------------------------------------------------------------------

void Kinematics::Joint2TaskUpdate(float theta[4]){
	float Q1 = (M_PI/180)*theta[0];
	float Q2 = (M_PI/180)*theta[1];
	float Q3 = (M_PI/180)*theta[2];
	float Q4 = (M_PI/180)*theta[3];

   float Q_1 = Q1;
   float Q_2 = Q2;
   float Q_3 = Q3;
   float Q_4 = Q4;
	float x = 135.0*cos(Q_1 - 1.0*Q_2) + 85.0*cos(Q_1 + Q_2 + Q_3 + Q_4) + 150.0*cos(Q_2 - 1.0*Q_1 + Q_3) + 135.0*cos(Q_1 + Q_2) + 85.0*cos(Q_2 - 1.0*Q_1 + Q_3 + Q_4) + 32.440000000002328306436538696289*cos(Q_1) + 150.0*cos(Q_1 + Q_2 + Q_3);
	float y = 135.0*sin(Q_1 - 1.0*Q_2) + 85.0*sin(Q_1 + Q_2 + Q_3 + Q_4) - 150.0*sin(Q_2 - 1.0*Q_1 + Q_3) + 135.0*sin(Q_1 + Q_2) - 85.0*sin(Q_2 - 1.0*Q_1 + Q_3 + Q_4) + 32.440000000002328306436538696289*sin(Q_1) + 150.0*sin(Q_1 + Q_2 + Q_3);
	float z = 300.0*sin(Q_2 + Q_3) + 270.0*sin(Q_2) + 170*sin(Q_2 + Q_3 + Q_4) + 108.0;

	_xyz[0] = x;
	_xyz[1] = y;
	_xyz[2] = z;

	return;
}

float Kinematics::Joint2TaskX(void){
	return _xyz[0];
}
float Kinematics::Joint2TaskY(void){
	return _xyz[1];
}
float Kinematics::Joint2TaskZ(void){
	return _xyz[2];
}

//------------------------------------------------------------------------------------
//------------------------REVERSE KINEMATICS------------------------------------------
//------------------------------------------------------------------------------------
float Kinematics::Task2JointQ1(double x, double y){
	float y_offset = 0;
	return (180/M_PI)*atan(y/x);

}

float Kinematics::Task2JointQ2(double x, double y, double z){

/*
	double _rwex = (0.5*cos(Q_1 + Q_2 + Q_3 + Q_4) + 0.5*cos(Q_2 - 1.0*Q_1 + Q_3 + Q_4))*170;
	double _rwey = (0.5*sin(Q_1 + Q_2 + Q_3 + Q_4) - 0.5*sin(Q_2 - 1.0*Q_1 + Q_3 + Q_4))*170;
	double _rwez = (sin(Q_2 + Q_3 + Q_4))*170;

	double _rowx = x - _rwez;
	double _rowy = y - _rwez;
	double _rowz = z - _rwez;

	double _r24x = -_rowx + L1X;
	double _r24y = -_rowy;
	double _r24z = -_rowz + L1Z;
*/
	double a = L3;
	double x_of = x - L1X;
	double z_of = z - L1Z;
	double b = sqrt(x_of*x_of + y*y + z_of*z_of);
	//Serial.println(b);
  	double c = L2;
	float y_offset = 0;
	return (180/M_PI)*(acos((b*b+c*c-a*a)/(2*b*c)));
}

float Kinematics::Task2JointQ3(double x, double y, double z){

	double a = L3;
	double b = sqrt(x*x + y*y + z*z);
  	double c = L2;
	return (180/M_PI)*(acos((a*a+b*b-b)/(2*a*c)))+180;
}

float Kinematics::ComputeDifferentialKinematics(float theta[4], float theta_dot[4]){
	float Q_1 = (M_PI/180)*theta[0];
	float Q_2 = (M_PI/180)*theta[1];
	float Q_3 = (M_PI/180)*theta[2];
	float Q_4 = (M_PI/180)*theta[3];
	float Q_1_dot = (M_PI/180)*theta_dot[0];
	float Q_2_dot = (M_PI/180)*theta_dot[1];
	float Q_3_dot = (M_PI/180)*theta_dot[2];
	float Q_4_dot = (M_PI/180)*theta_dot[3];


	float v_1r1e[3];
	v_1r1e[0] = (L1X + L2*cos(Q_2)+L3*cos(Q_1+Q_2)+L4*cos(Q_1+Q_2+Q_3))*Q_1_dot;
	v_1r1e[1] = 0;
	v_1r1e[2] = (L1Z + L2*sin(Q_2)+L3*sin(Q_1+Q_2)+L4*sin(Q_1+Q_2+Q_3))*Q_1_dot;

	float v_2r2e[3];
	v_2r2e[0] = (L2 + L3*cos(Q_2) + L4*cos(Q_2+Q_3))*Q_2_dot;
	v_2r2e[1] = (0 + L3*sin(Q_2) + L4*sin(Q_2+Q_3))*Q_2_dot;
	v_2r2e[2] = 0;

	float v_3r3e[3];
	v_3r3e[0] = (L3 + L4*cos(Q_3))*Q_3_dot;
	v_3r3e[1] = (0 + L4*sin(Q_3))*Q_3_dot;
	v_3r3e[2] = 0;

	float v_4r4e[3];
	v_4r4e[0] = L4*Q_4_dot;
	v_4r4e[1] = 0;
	v_4r4e[2] = 0;

	_xdot[0] = v_1r1e[0] + v_2r2e[0] + v_3r3e[0] + v_4r4e[0];					//vex
	_xdot[1] = v_1r1e[1] + v_2r2e[1] + v_3r3e[1] + v_4r4e[1];					//vey
	_xdot[2] = v_1r1e[2] + v_2r2e[2] + v_3r3e[2] + v_4r4e[2];					//vez
	_xdot[3] = Q_2_dot*sin(Q_1) + Q_3_dot*sin(Q_1) + Q_4_dot*sin(Q_1);			//wex
	_xdot[4] = -1*(Q_2_dot*cos(Q_1) + Q_3_dot*cos(Q_1) + Q_4_dot*cos(Q_1));		//wey
	_xdot[5] = Q_1_dot;															//wez


}
