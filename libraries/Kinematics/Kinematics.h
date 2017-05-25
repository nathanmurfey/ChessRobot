#ifndef Kinematics_h
#define Kinematics_h 

#include <Arduino.h>
//#include <MatrixMath.h>

class Kinematics
{

	public:
		Kinematics(void);
		void Joint2TaskUpdate (float[4]);
		void Task2Joint (double, double, double[3]);
		void Joint2TaskSimple(float[4]);
		float Task2JointQ1(double, double);
		float Task2JointQ2(double, double, double);
		float Task2JointQ3(double, double, double);
		float Joint2TaskX(void);
		float Joint2TaskY(void);
		float Joint2TaskZ(void);
		float ComputeDifferentialKinematics(float[4], float[4]);

		//int ar[2][2];
	private:
		float _xyz[3];
		float _xdot[6];
		//void MatrixMultiply(double[4][4],double[4][4],double[4][4]);

};

#endif