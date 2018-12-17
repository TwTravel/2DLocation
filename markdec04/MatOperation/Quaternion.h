#ifndef __QUATERNION_H_INCLUDED__
#define __QUATERNION_H_INCLUDED__
#include "MatMath.h"

class TVector3;
class EulerAngles;
class Quaternion;

class EulerAngles {
public:
// Public data
	// Straightforward representation.  Store the three angles, in
	// radians
	float	heading;
	float	pitch;
	float	bank;
// Public operations
	// Default constructor does nothing
	EulerAngles() {}
	// Construct from three values
	EulerAngles(float h, float p, float b) :
		heading(h), pitch(p), bank(b) {}
	// Set to identity triple (all zeros)
	void	identity() { pitch = bank = heading = 0.0f; }
	// Determine "canonical" Euler angle triple
	void	canonize();
	// Convert the quaternion to Euler angle format.  The input quaternion
	// is assumed to perform the rotation from object-to-inertial
	// or inertial-to-object, as indicated.
	void	fromObjectToInertialQuaternion(const Quaternion &q);
	void	fromInertialToObjectQuaternion(const Quaternion &q);
	// Convert the transform matrix to Euler angle format.  The input
	// matrix is assumed to perform the transformation from
	// object-to-world, or world-to-object, as indicated.  The
	// translation portion of the matrix is ignored.  The
	// matrix is assumed to be orthogonal.
	//void	fromObjectToWorldMatrix(const Matrix4x3 &m);
	//void	fromWorldToObjectMatrix(const Matrix4x3 &m);
	// Convert a rotation matrix to Euler Angle form.
	void	fromRotationMatrix(const RotationMatrix &m);
};


//---------------------------------------------------------------------------
// class Quaternion
//
// Implement a quaternion, for purposes of representing an angular
// displacement (orientation) in 3D.

class Quaternion {
public:

// Public data

	// The 4 values of the quaternion.  Normally, it will not
	// be necessary to manipulate these directly.  However,
	// we leave them public, since prohibiting direct access
	// makes some operations, such as file I/O, unnecessarily
	// complicated.

	float	w, x, y, z;

// Public operations

	// Set to identity

	void	identity() { w = 1.0f; x = y = z = 0.0f; }

	// Setup the quaternion to a specific rotation

	void	setToRotateAboutX(float theta);
	void	setToRotateAboutY(float theta);
	void	setToRotateAboutZ(float theta);
	void	setToRotateAboutAxis(const TVector3 &axis, float theta);

	// Setup to perform object<->inertial rotations,
	// given orientation in Euler angle format

	void	setToRotateObjectToInertial(const EulerAngles &orientation);
	void	setToRotateInertialToObject(const EulerAngles &orientation);

	// Cross product

	Quaternion operator *(const Quaternion &a) const;

	// Multiplication with assignment, as per C++ convention

	Quaternion &operator *=(const Quaternion &a);

	// Normalize the quaternion.

	void	normalize();

	// Extract and return the rotation angle and axis.

	float	getRotationAngle() const;
	TVector3	getRotationAxis() const;
};

void EulerAngles::fromObjectToInertialQuaternion(const Quaternion &q) {

	// Extract sin(pitch)
    const float kPi = 3.14159265f;
    const float k2Pi = kPi * 2.0f;
    const float kPiOver2 = kPi / 2.0f;

	float sp = -2.0f * (q.y*q.z - q.w*q.x);

	// Check for Gimbel lock, giving slight tolerance for numerical imprecision

	if (fabs(sp) > 0.9999f) {

		// Looking straight up or down

		pitch = kPiOver2 * sp;

		// Compute heading, slam bank to zero

		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;

	} else {

		// Compute angles.  We don't have to use the "safe" asin
		// function because we already checked for range errors when
		// checking for Gimbel lock

		pitch	= asin(sp);
		heading	= atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank	= atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}
// A global "identity" quaternion constant

extern const Quaternion kQuaternionIdentity;

// Quaternion dot product.

extern float dotProduct(const Quaternion &a, const Quaternion &b);

// Spherical linear interpolation

extern Quaternion slerp(const Quaternion &p, const Quaternion &q, float t);

// Quaternion conjugation

extern Quaternion conjugate(const Quaternion &q);

// Quaternion exponentiation

extern Quaternion pow(const Quaternion &q, float exponent);

/////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////
//
// global data
//
/////////////////////////////////////////////////////////////////////////////

// The global identity quaternion.  Notice that there are no constructors
// to the Quaternion class, since we really don't need any.

const Quaternion kQuaternionIdentity = {
	1.0f, 0.0f, 0.0f, 0.0f
};

/////////////////////////////////////////////////////////////////////////////
//
// class Quaternion members
//
/////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
// Quaternion::setToRotateAboutX
// Quaternion::setToRotateAboutY
// Quaternion::setToRotateAboutZ
// Quaternion::setToRotateAboutAxis
//
// Setup the quaternion to rotate about the specified axis

void	Quaternion::setToRotateAboutX(float theta) {

	// Compute the half angle

	float	thetaOver2 = theta * .5f;

	// Set the values

	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void	Quaternion::setToRotateAboutY(float theta) {

	// Compute the half angle

	float	thetaOver2 = theta * .5f;

	// Set the values

	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void	Quaternion::setToRotateAboutZ(float theta) {

	// Compute the half angle

	float	thetaOver2 = theta * .5f;

	// Set the values

	w = cos(thetaOver2);
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void	Quaternion::setToRotateAboutAxis(const TVector3 &axis, float theta) {

	// The axis of rotation must be normalized

	assert(fabs(vectorMag(axis) - 1.0f) < .01f);

	// Compute the half angle and its sin

	float	thetaOver2 = theta * .5f;
	float	sinThetaOver2 = sin(thetaOver2);

	// Set the values

	w = cos(thetaOver2);
	x = axis.X * sinThetaOver2;
	y = axis.Y * sinThetaOver2;
	z = axis.Z * sinThetaOver2;
}

//---------------------------------------------------------------------------
// EulerAngles::setToRotateObjectToInertial
//
// Setup the quaternion to perform an object->inertial rotation, given the
// orientation in Euler angle format
//
// See 10.6.5 for more information.

void	Quaternion::setToRotateObjectToInertial(const EulerAngles &orientation) {

	// Compute sine and cosine of the half angles

	float	sp, sb, sh;
	float	cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);

	// Compute values

	w =  ch*cp*cb + sh*sp*sb;
	x =  ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;
}

//---------------------------------------------------------------------------
// EulerAngles::setToRotateInertialToObject
//
// Setup the quaternion to perform an object->inertial rotation, given the
// orientation in Euler angle format
//
// See 10.6.5 for more information.

void	Quaternion::setToRotateInertialToObject(const EulerAngles &orientation) {

	// Compute sine and cosine of the half angles

	float	sp, sb, sh;
	float	cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);

	// Compute values

	w =  ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y =  ch*sp*sb - sh*cb*cp;
	z =  sh*sp*cb - ch*cp*sb;
}

//---------------------------------------------------------------------------
// Quaternion::operator *
//
// Quaternion cross product, which concatonates multiple angular
// displacements.  The order of multiplication, from left to right,
// corresponds to the order that the angular displacements are
// applied.  This is backwards from the *standard* definition of
// quaternion multiplication.  See section 10.4.8 for the rationale
// behind this deviation from the standard.

Quaternion Quaternion::operator *(const Quaternion &a) const {
	Quaternion result;

	result.w = w*a.w - x*a.x - y*a.y - z*a.z;
	result.x = w*a.x + x*a.w + z*a.y - y*a.z;
	result.y = w*a.y + y*a.w + x*a.z - z*a.x;
	result.z = w*a.z + z*a.w + y*a.x - x*a.y;

	return result;
}

//---------------------------------------------------------------------------
// Quaternion::operator *=
//
// Combined cross product and assignment, as per C++ convention

Quaternion &Quaternion::operator *=(const Quaternion &a) {

	// Multuiply and assign

	*this = *this * a;

	// Return reference to l-value

	return *this;
}

//---------------------------------------------------------------------------
// Quaternion::normalize
//
// "Normalize" a quaternion.  Note that normally, quaternions
// are always normalized (within limits of numerical precision).
// See section 10.4.6 for more information.
//
// This function is provided primarily to combat floating point "error
// creep," which can occur when many successive quaternion operations
// are applied.

void	Quaternion::normalize() {

	// Compute magnitude of the quaternion

	float	mag = (float)sqrt(w*w + x*x + y*y + z*z);

	// Check for bogus length, to protect against divide by zero

	if (mag > 0.0f) {

		// Normalize it

		float	oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;

	} else {

		// Houston, we have a problem

		assert(false);

		// In a release build, just slam it to something

		identity();
	}
}

//---------------------------------------------------------------------------
// Quaternion::getRotationAngle
//
// Return the rotation angle theta

float	Quaternion::getRotationAngle() const {

	// Compute the half angle.  Remember that w = cos(theta / 2)

	float thetaOver2 = safeAcos(w);

	// Return the rotation angle

	return thetaOver2 * 2.0f;
}

//---------------------------------------------------------------------------
// Quaternion::getRotationAxis
//
// Return the rotation axis

TVector3	Quaternion::getRotationAxis() const {

	// Compute sin^2(theta/2).  Remember that w = cos(theta/2),
	// and sin^2(x) + cos^2(x) = 1

	float sinThetaOver2Sq = 1.0f - w*w;

	// Protect against numerical imprecision

	if (sinThetaOver2Sq <= 0.0f) {

		// Identity quaternion, or numerical imprecision.  Just
		// return any valid vector, since it doesn't matter

		return TVector3(1.0f, 0.0f, 0.0f);
	}

	// Compute 1 / sin(theta/2)

	float	oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);

	// Return axis of rotation

	return TVector3(
		x * oneOverSinThetaOver2,
		y * oneOverSinThetaOver2,
		z * oneOverSinThetaOver2
	);
}

/////////////////////////////////////////////////////////////////////////////
//
// Nonmember functions
//
/////////////////////////////////////////////////////////////////////////////

//---------------------------------------------------------------------------
// dotProduct
//
// Quaternion dot product.  We use a nonmember function so we can
// pass quaternion expressions as operands without having "funky syntax"
//
// See 10.4.10

float dotProduct(const Quaternion &a, const Quaternion &b) {
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

//---------------------------------------------------------------------------
// slerp
//
// Spherical linear interpolation.
//
// See 10.4.13

Quaternion slerp(const Quaternion &q0, const Quaternion &q1, float t) {

	// Check for out-of range parameter and return edge points if so

	if (t <= 0.0f) return q0;
	if (t >= 1.0f) return q1;

	// Compute "cosine of angle between quaternions" using dot product

	float cosOmega = dotProduct(q0, q1);

	// If negative dot, use -q1.  Two quaternions q and -q
	// represent the same rotation, but may produce
	// different slerp.  We chose q or -q to rotate using
	// the acute angle.

	float q1w = q1.w;
	float q1x = q1.x;
	float q1y = q1.y;
	float q1z = q1.z;
	if (cosOmega < 0.0f) {
		q1w = -q1w;
		q1x = -q1x;
		q1y = -q1y;
		q1z = -q1z;
		cosOmega = -cosOmega;
	}

	// We should have two unit quaternions, so dot should be <= 1.0

	assert(cosOmega < 1.1f);

	// Compute interpolation fraction, checking for quaternions
	// almost exactly the same

	float k0, k1;
	if (cosOmega > 0.9999f) {

		// Very close - just use linear interpolation,
		// which will protect againt a divide by zero

		k0 = 1.0f-t;
		k1 = t;

	} else {

		// Compute the sin of the angle using the
		// trig identity sin^2(omega) + cos^2(omega) = 1

		float sinOmega = sqrt(1.0f - cosOmega*cosOmega);

		// Compute the angle from its sin and cosine

		float omega = atan2(sinOmega, cosOmega);

		// Compute inverse of denominator, so we only have
		// to divide once

		float oneOverSinOmega = 1.0f / sinOmega;

		// Compute interpolation parameters

		k0 = sin((1.0f - t) * omega) * oneOverSinOmega;
		k1 = sin(t * omega) * oneOverSinOmega;
	}

	// Interpolate

	Quaternion result;
	result.x = k0*q0.x + k1*q1x;
	result.y = k0*q0.y + k1*q1y;
	result.z = k0*q0.z + k1*q1z;
	result.w = k0*q0.w + k1*q1w;

	// Return it

	return result;
}

//---------------------------------------------------------------------------
// conjugate
//
// Compute the quaternion conjugate.  This is the quaternian
// with the opposite rotation as the original quaternian.  See 10.4.7

Quaternion conjugate(const Quaternion &q) {
	Quaternion result;

	// Same rotation amount

	result.w = q.w;

	// Opposite axis of rotation

	result.x = -q.x;
	result.y = -q.y;
	result.z = -q.z;

	// Return it

	return result;
}

//---------------------------------------------------------------------------
// pow
//
// Quaternion exponentiation.
//
// See 10.4.12

Quaternion pow(const Quaternion &q, float exponent) {

	// Check for the case of an identity quaternion.
	// This will protect against divide by zero

	if (fabs(q.w) > .9999f) {
		return q;
	}

	// Extract the half angle alpha (alpha = theta/2)

	float	alpha = acos(q.w);

	// Compute new alpha value

	float	newAlpha = alpha * exponent;

	// Compute new w value

	Quaternion result;
	result.w = cos(newAlpha);

	// Compute new xyz values

	float	mult = sin(newAlpha) / sin(alpha);
	result.x = q.x * mult;
	result.y = q.y * mult;
	result.z = q.z * mult;

	// Return it

	return result;
}
#endif // #ifndef __QUATERNION_H_INCLUDED__
