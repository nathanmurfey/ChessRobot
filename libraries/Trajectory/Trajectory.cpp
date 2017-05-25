#include "Trajectory.h"
#include "HardwareSerial.h"
#include <MatrixMath.h>


//Initial values:
//float initialVal[6] = {pos_xi, pos_yi, pos_zi, vel_xi, vel_yi, vel_zi};

//Final values:
//float finalVal[6] = {pos_xf, pos_yf, pos_zf, vel_xf, vel_yf, vel_zf};

//Coefficients for polynomials:
float _xCoeff[4] = {0,0,0,0};
float _yCoeff[4] = {0,0,0,0};
float _zCoeff[4] = {0,0,0,0};
//For each time increment, get velocity:
float velX;
float velY;
float velZ;


//float getVelocity(float, float, float, int); //three coefficients and time at instant called

Trajectory::Trajectory(void)
{
	
}


void Trajectory::TrajectoryBuild(float initialVal[6], float finalVal[6], float tf){
	//Solve for polynomial coefficients:

	for (int i = 0; i < 6; i++){
		initialVal[i] =  initialVal[i]/1000;
		finalVal[i] = finalVal[i]/1000;

	}

	_xCoeff[0] = initialVal[0]; //pos_xi
	_xCoeff[1] = initialVal[3]; //vel_xi
	_xCoeff[2] = (-1/tf)*(finalVal[3] + 2*_xCoeff[1] + (3/tf)*_xCoeff[0] - (3/tf)*finalVal[0]);
	_xCoeff[3] = finalVal[0]/(tf*tf*tf) - _xCoeff[0]/(tf*tf*tf) - _xCoeff[1]/(tf*tf) + (1/(tf*tf))*(finalVal[3] + 2*_xCoeff[1] + (3/tf)*_xCoeff[0] - (3/tf)*finalVal[0]);

	_yCoeff[0] = initialVal[1]; //pos_xi
	_yCoeff[1] = initialVal[4]; //vel_xi
	_yCoeff[2] = (-1/tf)*(finalVal[4] + 2*_yCoeff[1] + (3/tf)*_yCoeff[0] - (3/tf)*finalVal[1]);
	_yCoeff[3] = finalVal[1]/(tf*tf*tf) - _yCoeff[0]/(tf*tf*tf) - _yCoeff[1]/(tf*tf) + (1/(tf*tf))*(finalVal[4] + 2*_yCoeff[1] + (3/tf)*_yCoeff[0] - (3/tf)*finalVal[1]);

	_zCoeff[0] = initialVal[2]; //pos_xi
	_zCoeff[1] = initialVal[5]; //vel_xi
	_zCoeff[2] = (-1/tf)*(finalVal[5] + 2*_zCoeff[1] + (3/tf)*_zCoeff[0] - (3/tf)*finalVal[2]);
	_zCoeff[3] = finalVal[2]/(tf*tf*tf) - _zCoeff[0]/(tf*tf*tf) - _zCoeff[1]/(tf*tf) + (1/(tf*tf))*(finalVal[5] + 2*_zCoeff[1] + (3/tf)*_zCoeff[0] - (3/tf)*finalVal[2]);

}

void Trajectory::TrajectoryUpdate(void){
	//For each time increment, get velocity:
    velX = getVelocity(_xCoeff[1], _xCoeff[2], _xCoeff[3], millis());
    velY = getVelocity(_yCoeff[1], _yCoeff[2], _yCoeff[3], millis());
    velZ = getVelocity(_zCoeff[1], _zCoeff[2], _zCoeff[3], millis());
}
float Trajectory::getVelocityX(void){
	return velX;
}
float Trajectory::getVelocityY(void){
	return velY;
}
float Trajectory::getVelocityZ(void){
	return velZ;
}

float Trajectory::getVelocity(float a1, float a2, float a3, int timeMS){
  float t = 0.001*timeMS;
  return (a1 + 2*a2*t + 3*a3*t*t);
}

