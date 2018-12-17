/* Some simple mathematical functions. Don't look for some logic in
   the function names :-) */
#ifndef __MatMatrix_H_INCLUDED__
#define __MatMatrix_H_INCLUDED__
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Matrix.h"

inline void sinCos(float *returnSin, float *returnCos, float theta) {

	// For simplicity, we'll just use the normal trig functions.
	// Note that on some platforms we may be able to do better

	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

inline float safeAcos(float x) {

        // Check limit conditions
        if (x <= -1.0f) {
                return 3.1415926;//kPi;
        }
        if (x >= 1.0f) {
                return 0.0f;
        }
        // Value is in the domain - use standard C function
        return acos(x);
}


class TMat4
{
 public:
 TMat4(){ m.SetDimension(4,4);};
 Matrix<double> m;
};

typedef  TMat4 RotationMatrix;

class TMat3
{
 public:
 TMat3(){ m.SetDimension(3,3);};
 Matrix<double> m;
};


class Matrix4x3
{
 public:
 Matrix4x3(){ m.SetDimension(4,3);};
 Matrix<double> m;
};

class TVector3
{
  public:
  TVector3(){}
  TVector3(double inX, double inY, double inZ)
  { X = inX; Y = inY; Z = inZ; }
 // (1.0f, 0.0f, 0.0f)
  double X,Y,Z;
};


inline float vectorMag(const TVector3 &a) {
	return sqrt(a.X*a.X + a.Y*a.Y + a.Z*a.Z);
}

// Compute the cross product of two vectors

inline TVector3 crossProduct(const TVector3 &a, const TVector3 &b) {
	return TVector3(
		a.Y*b.Z - a.Z*b.Y,
		a.Z*b.X - a.X*b.Z,
		a.X*b.Y - a.Y*b.X
	);
}

// Scalar on the left multiplication, for symmetry

inline TVector3 operator *(float k, const TVector3 &v) {
	return TVector3(k*v.X, k*v.Y, k*v.Z);
}

// Compute the distance between two points

inline float distance(const TVector3 &a, const TVector3 &b) {
	float dx = a.X - b.X;
	float dy = a.Y - b.Y;
	float dz = a.Z - b.Z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}

// Compute the distance between two points, squared.  Often useful
// when comparing distances, since the square root is slow

inline float distanceSquared(const TVector3 &a, const TVector3 &b) {
	float dx = a.X - b.X;
	float dy = a.Y - b.Y;
	float dz = a.Z - b.Z;
	return dx*dx + dy*dy + dz*dz;
}


class TVector4
{
  public:
  TVector4(){}
  TVector4(double inX, double inY, double inZ, double inW)
  { X = inX; Y = inY; Z = inZ; W = inW; }
  double X,Y,Z,W;
};

/* ******* Gestion des matrices 4x4 ****** */

void inline gl_M4_Id(TMat4 *a)
{
	int i,j;
	for(i=0;i<4;i++)
	for(j=0;j<4;j++) 
	if (i==j) a->m[i][j]=1.0; else a->m[i][j]=0.0;
}

int gl_M4_IsId(TMat4 *a)
{
	int i,j;
	for(i=0;i<4;i++)
    for(j=0;j<4;j++) {
      if (i==j) { 
        if (a->m[i][j] != 1.0) return 0;
      } else if (a->m[i][j] != 0.0) return 0;
    }
  return 1;
}

void inline gl_M4_Mul(TMat4 *c,TMat4 *a,TMat4 *b)
{
  int i,j,k;
  double s;
  for(i=0;i<4;i++)
    for(j=0;j<4;j++) {
      s=0.0;
      for(k=0;k<4;k++) s+=a->m[i][k]*b->m[k][j];
      c->m[i][j]=s;
    }
}

/* c=c*a */
void inline gl_M4_MulLeft(TMat4 *c,TMat4 *b)
{
  int i,j,k;
  double s;
  TMat4 a;

  /*memcpy(&a, c, 16*sizeof(double));
  */
  a=*c;

  for(i=0;i<4;i++)
    for(j=0;j<4;j++) {
      s=0.0;
      for(k=0;k<4;k++) s+=a.m[i][k]*b->m[k][j];
      c->m[i][j]=s;
    }
}

void inline gl_M4_Move(TMat4 *a,TMat4 *b)
{
	memcpy(a,b,sizeof(TMat4));
}

 


void inline gl_MulM4V3(TVector3 *a,TMat4 *b,TVector3 *c)
{
	 a->X=b->m[0][0]*c->X+b->m[0][1]*c->Y+b->m[0][2]*c->Z+b->m[0][3];
	 a->Y=b->m[1][0]*c->X+b->m[1][1]*c->Y+b->m[1][2]*c->Z+b->m[1][3];
	 a->Z=b->m[2][0]*c->X+b->m[2][1]*c->Y+b->m[2][2]*c->Z+b->m[2][3];
}

void inline gl_MulM3V3(TVector3 *a,TMat3 *b,TVector3 *c)
{
	 a->X=b->m[0][0]*c->X+b->m[0][1]*c->Y+b->m[0][2]*c->Z;
	 a->Y=b->m[1][0]*c->X+b->m[1][1]*c->Y+b->m[1][2]*c->Z;
	 a->Z=b->m[2][0]*c->X+b->m[2][1]*c->Y+b->m[2][2]*c->Z;
}

void inline gl_M4_MulV4(TVector4 *a,TMat4 *b,TVector4 *c)
{
	 a->X=b->m[0][0]*c->X+b->m[0][1]*c->Y+b->m[0][2]*c->Z+b->m[0][3]*c->W;
	 a->Y=b->m[1][0]*c->X+b->m[1][1]*c->Y+b->m[1][2]*c->Z+b->m[1][3]*c->W;
	 a->Z=b->m[2][0]*c->X+b->m[2][1]*c->Y+b->m[2][2]*c->Z+b->m[2][3]*c->W;
	 a->W=b->m[3][0]*c->X+b->m[3][1]*c->Y+b->m[3][2]*c->Z+b->m[3][3]*c->W;
}
	
/* transposition of a 4x4 matrix */
void inline gl_M4_Transpose(TMat4 *a,TMat4 *b)
{
  a->m[0][0]=b->m[0][0]; 
  a->m[0][1]=b->m[1][0]; 
  a->m[0][2]=b->m[2][0]; 
  a->m[0][3]=b->m[3][0]; 

  a->m[1][0]=b->m[0][1]; 
  a->m[1][1]=b->m[1][1]; 
  a->m[1][2]=b->m[2][1]; 
  a->m[1][3]=b->m[3][1]; 

  a->m[2][0]=b->m[0][2]; 
  a->m[2][1]=b->m[1][2]; 
  a->m[2][2]=b->m[2][2]; 
  a->m[2][3]=b->m[3][2]; 

  a->m[3][0]=b->m[0][3]; 
  a->m[3][1]=b->m[1][3]; 
  a->m[3][2]=b->m[2][3]; 
  a->m[3][3]=b->m[3][3]; 
}

/* inversion of an orthogonal matrix of type Y=M.X+P */ 
void inline gl_M4_InvOrtho(TMat4 *a,TMat4 b)
{
	int i,j;
	double s;
	for(i=0;i<3;i++)
	for(j=0;j<3;j++) a->m[i][j]=b.m[j][i];
	a->m[3][0]=0.0; a->m[3][1]=0.0; a->m[3][2]=0.0; a->m[3][3]=1.0;
	for(i=0;i<3;i++) {
		s=0;
		for(j=0;j<3;j++) s-=b.m[j][i]*b.m[j][3];
		a->m[i][3]=s;
	}
}

/* Inversion of a general nxn matrix.
   Note : m is destroyed */

int inline Matrix_Inv(double *r,double *m,int n)
{
	 int i,j,k,l;
	 double max,tmp,t;

	 /* identitée dans r */
	 for(i=0;i<n*n;i++) r[i]=0;
	 for(i=0;i<n;i++) r[i*n+i]=1;
	 
	 for(j=0;j<n;j++) {
			
			/* recherche du nombre de plus grand module sur la colonne j */
			max=m[j*n+j];
			k=j;
			for(i=j+1;i<n;i++)
				if (fabs(m[i*n+j])>fabs(max)) {
					 k=i;
					 max=m[i*n+j];
				}

      /* non intersible matrix */
      if (max==0) return 1;

			
			/* permutation des lignes j et k */
			if (k!=j) {
				 for(i=0;i<n;i++) {
						tmp=m[j*n+i];
						m[j*n+i]=m[k*n+i];
						m[k*n+i]=tmp;
						
						tmp=r[j*n+i];
						r[j*n+i]=r[k*n+i];
						r[k*n+i]=tmp;
				 }
			}
			
			/* multiplication de la ligne j par 1/max */
			max=1/max;
			for(i=0;i<n;i++) {
				 m[j*n+i]*=max;
				 r[j*n+i]*=max;
			}
			
			
			for(l=0;l<n;l++) if (l!=j) {
				 t=m[l*n+j];
				 for(i=0;i<n;i++) {
						m[l*n+i]-=m[j*n+i]*t;
						r[l*n+i]-=r[j*n+i]*t;
				 }
			}
	 }

	 return 0;
}


/* inversion of a 4x4 matrix */

void inline gl_M4_Inv(TMat4 *a,TMat4 *b)
{
  TMat4 tmp;
  memcpy(&tmp, b, 16*sizeof(double));
  /*tmp=*b;*/
  Matrix_Inv(&a->m[0][0],&tmp.m[0][0],4);
}

void inline gl_M4_Rotate(TMat4 *a,double t,int u)
{
	 double s,c;
	 int v,w;
   if ((v=u+1)>2) v=0;
	 if ((w=v+1)>2) w=0;
	 s=sin(t);
	 c=cos(t);
	 gl_M4_Id(a);
	 a->m[v][v]=c;	a->m[v][w]=-s;
	 a->m[w][v]=s;	a->m[w][w]=c;
}
	

/* inverse of a 3x3 matrix */
void inline gl_M3_Inv(TMat3 *a,TMat3 *m)
{
	 double det;
	 
	 det = m->m[0][0]*m->m[1][1]*m->m[2][2]-m->m[0][0]*m->m[1][2]*m->m[2][1]-
		 m->m[1][0]*m->m[0][1]*m->m[2][2]+m->m[1][0]*m->m[0][2]*m->m[2][1]+
		 m->m[2][0]*m->m[0][1]*m->m[1][2]-m->m[2][0]*m->m[0][2]*m->m[1][1];

	 a->m[0][0] = (m->m[1][1]*m->m[2][2]-m->m[1][2]*m->m[2][1])/det;
	 a->m[0][1] = -(m->m[0][1]*m->m[2][2]-m->m[0][2]*m->m[2][1])/det;
	 a->m[0][2] = -(-m->m[0][1]*m->m[1][2]+m->m[0][2]*m->m[1][1])/det;
	 
	 a->m[1][0] = -(m->m[1][0]*m->m[2][2]-m->m[1][2]*m->m[2][0])/det;
	 a->m[1][1] = (m->m[0][0]*m->m[2][2]-m->m[0][2]*m->m[2][0])/det;
	 a->m[1][2] = -(m->m[0][0]*m->m[1][2]-m->m[0][2]*m->m[1][0])/det;

	 a->m[2][0] = (m->m[1][0]*m->m[2][1]-m->m[1][1]*m->m[2][0])/det;
	 a->m[2][1] = -(m->m[0][0]*m->m[2][1]-m->m[0][1]*m->m[2][0])/det;
	 a->m[2][2] = (m->m[0][0]*m->m[1][1]-m->m[0][1]*m->m[1][0])/det;
}

																										
/* vector arithmetic */

int inline gl_V3_Norm(TVector3 *a)
{
	double n;
	n=sqrt(a->X*a->X+a->Y*a->Y+a->Z*a->Z);
	if (n==0) return 1;
	a->X/=n;
	a->Y/=n;
	a->Z/=n;
	return 0;
}

TVector3 inline gl_V3_New(double x,double y,double z)
{
	 TVector3 a;
	 a.X=x;
	 a.Y=y;
	 a.Z=z;
	 return a;
}

TVector4 inline gl_V4_New(double x,double y,double z,double w)
{
  TVector4 a;
  a.X=x;
  a.Y=y;
  a.Z=z;
  a.W=w;
  return a;
}

#endif

