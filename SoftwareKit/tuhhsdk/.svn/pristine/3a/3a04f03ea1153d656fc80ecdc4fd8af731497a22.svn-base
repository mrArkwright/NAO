#ifndef __RotationMatrix_h__
#define __RotationMatrix_h__

#include "Matrix3x3.h"

/// Representation of a 3x3 Rotationmatrix
/**
 * This class represents a RotationMatrix
 * Inspired by <a href="http://www.b-human.de/file_download/32/bhuman10_coderelease.tar.bz2">BHuman Code-Release 2010</a>
 *
 * modified and extended by <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class RotationMatrix : public Matrix3x3<float>
{
public:

	/** default Constructor */
	RotationMatrix(){}

	/** constructor with columnwise initialization */  
	RotationMatrix(const Vector3<float>& c0, const Vector3<float>& c1, const Vector3<float>& c2):
		Matrix3x3<float>(c0,c1,c2)	{}

	
	/**
	* Copy constructor.
	* @param  other  The other matrix that is copied to this one
	*/
	RotationMatrix(const Matrix3x3<float>& other):
		Matrix3x3<float>(other){}

	/** constructor with elementwise initialization */
	RotationMatrix(	const float& m11, const float& m12, const float& m13,
					const float& m21, const float& m22, const float& m23,
					const float& m31, const float& m32, const float& m33
					):
		Matrix3x3<float>(	m11,m12,m13,
							m21,m22,m23,
							m31,m32,m33
						)	{}

	/** returns a rotation matrix about the x-axis (roll) 
	* @param alpha angle to rotate
	* @return RotX
	*/
	static RotationMatrix rotX(float alpha){
		return RotationMatrix(
			1.0f, 0.0f,			0.0f,
			0.0f, cos(alpha),	-sin(alpha),
			0.0f, sin(alpha),	cos(alpha)
			);
	}


	/** returns a rotation matrix about the y-axis (pitch) 
	* @param alpha angle to rotate
	* @return RotY
	*/
	static RotationMatrix rotY(float alpha){
		return RotationMatrix(
			cos(alpha),		0.0f,		sin(alpha),
			0.0f,			1.0f,		0.0f,
			-sin(alpha),	0.0f,		cos(alpha)
			);
	}


	/** returns a rotation matrix about the z-axis (yaw) 
	* @param alpha angle to rotate
	* @return RotZ
	*/
	static RotationMatrix rotZ(float alpha){
		return RotationMatrix(
			cos(alpha),		-sin(alpha),	0.0f,
			sin(alpha),		cos(alpha),		0.0f,
			0.0f,			0.0f,			1.0f
			);
	}


	/** returns the inverted RotationMatrix. Note that inverting RotationMatrices
	* is the same as transposing them
	* @return inverse
	*/
	RotationMatrix invert() const
	{
		return transpose();
	}
};
#endif