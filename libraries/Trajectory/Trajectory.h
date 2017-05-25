#ifndef Trajectory_h
#define Trajectory_h 

#include <Arduino.h>
//#include <MatrixMath.h>

class Trajectory
{

	public:
		Trajectory(void);
		void TrajectoryBuild(float[6], float[6], float);
		void TrajectoryUpdate(void);
		float getVelocityX(void);
		float getVelocityY(void);
		float getVelocityZ(void);
		float getVelocity(float, float, float, int);
	private:
		float _xCoeff[4];
		float _yCoeff[4];
		float _zCoeff[4];

};

#endif