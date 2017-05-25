#ifndef Jacobian_h
#define Jacobian_h 

#include <Arduino.h>
//#include <MatrixMath.h>

class Jacobian
{

	public:
		Jacobian(void);
		void ReturnInverseJacobian(float*);
		void ReturnJacobian(float*);
		void UpdateInverseJacobian(float[4]);
		void Calculate(void);
		void InputToMatrix(void);
		void UpdateSinCosine(float[4]);
	private:


};

#endif