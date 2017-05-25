#include "Jacobian.h"
#include "HardwareSerial.h"
#include <MatrixMath.h>
// -----------------------------------------------------------------------------------------------
// ------------------------------- PARAMS --------------------------------------------------------
// -----------------------------------------------------------------------------------------------

#define a_0 = 100;
#define N           4 //transformation matrix size
//#define PI 3.14159

const float L4 = 170.00;
const float L3 = 300.00;
const float L2 = 270.00;
const float L1Z = 77.55;
const float L1X = 32.44;

float Q_1 = PI/6;
float Q_2 = PI/3;
float Q_3 = PI/2;
float Q_4 = PI/4;

float timeStep, currTime, prevTime;

float s2_13, s1_2, s12, s2_134, s1, s123, s1234;
float c1_2, c1234, c2_13, c12, c2_134, c1, c123, c23, c2, c234;
float J11, J12, J13, J14, J21, J22, J23, J24, J31, J32, J33, J34;

float J[3][4];
float J_T[4][3];
float JJ_T[3][3];
float J_inv[4][3];
float I = 1.0;

//float JInvRet[4][3];

const int INV_JAC_ROWS = 4;
const int INV_JAC_COLS = 3;

Jacobian::Jacobian(void)
{

}

//------------------------------------------------------------------------------------
//------------------------FORWARD KINEMATICS------------------------------------------
//------------------------------------------------------------------------------------

void Jacobian::UpdateInverseJacobian(float theta[4]){
  
  // build matrix
  UpdateSinCosine(theta);
  Calculate();
  InputToMatrix();

  // do matrix math to invert
  Matrix.Multiply((float*)J, (float*)J_T, 3, 4, 3, (float*)JJ_T);
  Matrix.Invert((float*)JJ_T, 3);
  Matrix.Multiply((float*)J_T, (float*)JJ_T, 4, 3, 3, (float*)J_inv);

}

void Jacobian::ReturnInverseJacobian(float* JInvRet){
	
	float* A = (float*)J_inv;

	int m = 4;
	int n = 3;
	
	for (int i=0;i<m;i++)
        for(int j=0;j<n;j++)
        {
        	//JInvRet[i][j] = A[i][j];
            JInvRet[n*i+j] = A[n*i+j];
        }


}
void Jacobian::ReturnJacobian(float* JRet){
	
	float* A = (float*)J;

	int m = 3;
	int n = 4;
	
	for (int i=0;i<m;i++)
        for(int j=0;j<n;j++)
        {
        	//JInvRet[i][j] = A[i][j];
            JRet[n*i+j] = A[n*i+j];
        }


}

void Jacobian::Calculate(void){
  J11 = 150.0*s2_134 - 85.0*s2_13 - 135.0*s1_2 - 135.0*s12 + 85.0*s2_134 - 32.44*s1 - 150.0*s123;
  J12 = 135.0*s1_2 - 85.0*s1234 - 150.0*s2_13- 135.0*s12 - 85.0*s2_134 - 150.0*s123;
  J13 = -85.0*s1234 - 150.0*s2_13 - 85.0*s2_134 - 150.0*s123;
  J14 = -85.0*s1234 - 85.0*s2_134;
  
  J21 = 135.0*c1_2 + 85.0*c1234 + 150.0*c2_13 + 135.0*c12 + 85.0*c2_134 + 32.44*c1 + 150.0*c123;
  J22 = 85.0*c1234 - 135.0*c1_2 - 150.0*c2_13 + 135.0*c12 - 85.0*c2_134 + 150.0*c123;
  J23 = 85.0*c1234 - 150.0*c2_13 - 85.0*c2_134 + 150.0*c123;
  J24 = 85.0*c1234- 85.0*c2_134;
  
  J31 = 0;
  J32 = 300.0*c23 + 270.0*c2 + 170.0*c234;
  J33 = 300.0*c23 + 170.0*c234;
  J34 = 170.0*c234;
}

void Jacobian::UpdateSinCosine(float theta[4]){
  float Q_1 = theta[0];
  float Q_2 = theta[1];
  float Q_3 = theta[3];
  float Q_4 = theta[4]; 

  s2_13 = sin(Q_2 - 1.0*Q_1 + Q_3);
  s1_2 = sin(Q_1 - 1.0*Q_2);
  s12 = sin(Q_1 + Q_2);
  s2_134 = sin(Q_2 - 1.0*Q_1 + Q_3 + Q_4) ;
  s1 = sin(Q_1) ;
  s123 = sin(Q_1 + Q_2 + Q_3);
  s1234 = sin(Q_1 + Q_2 + Q_3 + Q_4) ;
  
  c1_2 = cos(Q_1 - 1.0*Q_2);
  c1234 = cos(Q_1 + Q_2 + Q_3 + Q_4);
  c2_13 = cos(Q_2 - 1.0*Q_1 + Q_3);
  c12 = cos(Q_1 + Q_2) ;
  c2_134 = cos(Q_2 - 1.0*Q_1 + Q_3 + Q_4);
  c1 = cos(Q_1);
  c123 = cos(Q_1 + Q_2 + Q_3);
  c23 = cos(Q_2 + Q_3);
  c2 = cos(Q_2);
  c234 = cos(Q_2 + Q_3 + Q_4);
}

void Jacobian::InputToMatrix(void){
  J[0][0] = J11;
  J[0][1] = J12;
  J[0][2] = J13;
  J[0][3] = J14;
  J[1][0] = J21;
  J[1][1] = J22;
  J[1][2] = J23;
  J[1][3] = J24;
  J[2][0] = J31;
  J[2][1] = J32;
  J[2][2] = J33;
  J[2][3] = J34;

  J_T[0][0] = J11;
  J_T[0][1] = J21;
  J_T[0][2] = J31;
  J_T[1][0] = J12;
  J_T[1][1] = J22;
  J_T[1][2] = J32;
  J_T[2][0] = J13;
  J_T[2][1] = J23;
  J_T[2][2] = J33;
  J_T[3][0] = J14;
  J_T[3][1] = J24;
  J_T[3][2] = J34;
}
