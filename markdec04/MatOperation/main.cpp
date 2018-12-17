#include <stdio.h>
#include <stdlib.h>
#include "Matrix.h"
#include "MatMath.h"
#include "Quaternion.h"


void Quaternion2RotMatrix(const Quaternion &q, Matrix<double>&m)
 {
    m.SetDimension(3,3);
	m[0][0] = 1.0f - 2.0f * (q.y*q.y + q.z*q.z);                                         
	m[0][1] = 2.0f * (q.x*q.y + q.w*q.z);                                                
	m[0][2] = 2.0f * (q.x*q.z - q.w*q.y);                                                
                                                                                       
	m[1][0] = 2.0f * (q.x*q.y - q.w*q.z);                                                
	m[1][1] = 1.0f - 2.0f * (q.x*q.x + q.z*q.z);                                         
	m[1][2] = 2.0f * (q.y*q.z + q.w*q.x);                                                
                                                                                       
	m[2][0] = 2.0f * (q.x*q.z + q.w*q.y);                                                
	m[2][1] = 2.0f * (q.y*q.z - q.w*q.x);                                                
	m[2][2] = 1.0f - 2.0f * (q.x*q.x + q.y*q.y);                                         
}

int main(int argc,char* argv[])
{
	Matrix <double> Mat(8,8);
	printf("hello world\n");
	Mat[3][1] =2;
	printf("%.2lf\n", Mat(3,1));
	return 1;
}
