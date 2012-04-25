#ifndef __Matrix3x3_h__
#define __Matrix3x3_h__

#include "Vector3.h"

/// Representation of a 3x3 Matrix
/**
 * This class represents a 3x3-matrix.\n
 * Inspired by <a href="http://www.b-human.de/file_download/32/bhuman10_coderelease.tar.bz2">BHuman Code-Release 2010</a>
 *
 * modified and extended by <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 *
 */
template<class T = float> class Matrix3x3
{
public:
	/** three column vectors */
	Vector3<T> c[3];

	/** default constructor, creates a 3x3 identity matrix*/
	Matrix3x3<T>()		
	{
		c[0] = Vector3<T>(1,0,0);
		c[1] = Vector3<T>(0,1,0);
		c[2] = Vector3<T>(0,0,1);
	}

	/** copy-constructor */
	Matrix3x3<T>(const Matrix3x3<T>& other)
	{
		*this = other;
	}   

	/** constructor with elementwise initialization */
	Matrix3x3<T>(
		const T& m11, const T& m12, const T& m13,
		const T& m21, const T& m22, const T& m23,
		const T& m31, const T& m32, const T& m33)
	{
		c[0] = Vector3<T>(m11,m21,m31);
		c[1] = Vector3<T>(m12,m22,m32);
		c[2] = Vector3<T>(m13,m23,m33);
	}

	/** constructor with columnwise initialization */
	Matrix3x3<T>(const Vector3<T>& c0, const Vector3<T>& c1, const Vector3<T>& c2) 
	{
		c[0]=c0;
		c[1]=c1;
		c[2]=c2;
	}

	/** returns the transposed Matrix 
	* @return transposed
	*/
	Matrix3x3<T> transpose() const
	{
		return Matrix3x3<T>(
			Vector3<T>(c[0].x, c[1].x, c[2].x),
			Vector3<T>(c[0].y, c[1].y, c[2].y),
			Vector3<T>(c[0].z, c[1].z, c[2].z)
		);		
	}

	/** adds a Matrix3x3 to this one 
	* @param other other Rotationmatrix
	* @return sum
	*/
	Matrix3x3<T>& operator+=(const Matrix3x3<T>& other)
	{
		c[0] += other.c[0];
		c[1] += other.c[1];
		c[2] += other.c[2];
		return *this;
	}

	/** adds a Matrix3x3 to this one 
	* @param other other Matrix3x3
	* @return sum
	*/
	Matrix3x3<T> operator+(const Matrix3x3<T>& other) const
	{
		return Matrix3x3<T>(*this) += other;
	}

	/** subtracts a Matrix3x3 from this one 
	* @param other Matrix3x3 which shall be subtracted from this one (subtrahend)
	* @return difference
	*/
	Matrix3x3<T>& operator-=(const Matrix3x3<T>& other)
	{
		c[0] -= other.c[0];
		c[1] -= other.c[1];
		c[2] -= other.c[2];
		return *this;
	}

	/** subtracts a Matrix3x3 from this one 
	* @param other Matrix3x3 which shall be subtracted from this one (subtrahend)
	* @return difference
	*/
	Matrix3x3<T> operator-(const Matrix3x3<T>& other) const
	{
		return Matrix3x3<T>(*this) -= other;
	}

	/** multiplies this Matrix3x3 by a factor
	* @param factor
	* @return product
	*/
	Matrix3x3<T>& operator*=(const T& factor)
	{
		c[0] *= factor;
		c[1] *= factor;
		c[2] *= factor;
		return *this;
	}

	/** multiplication by a factor
	* @param factor to multiply
	* @return product
	*/
	Matrix3x3<T> operator*(const T& factor) const
	{
		return Matrix3x3<T>(*this) *= factor;
	}


	/** multiplies this Matrix3x3 by a vector
	* @param vector
	* @return product
	*/
	Vector3<T> operator*(const Vector3<T>& vector) const
	{
		return (c[0]*vector.x + c[1]*vector.y +c[2]*vector.z);
	}

	/** multiplication by another Matrix3x3
	* @param other Matrix3x3
	* @return product
	*/
	Matrix3x3<T> operator*(const Matrix3x3<T>& other) const
	{
		Vector3<T> v1 = *this*other.c[0];
		Vector3<T> v2 = *this*other.c[1];
		Vector3<T> v3 = *this*other.c[2];
		
		return Matrix3x3<T>(v1,v2,v3);
		
	}	


	/** multiplication by another Matrix3x3
	* @param other Matrix3x3
	* @return product
	*/
	Matrix3x3<T>& operator*=(const Matrix3x3<T>& other)
	{
		return *this = *this * other;
	}


	/** divides a Matrix3x3 through a factor
	* @param factor
	* @return quotient
	*/
	Matrix3x3<T>& operator/=(const T& factor)
	{
		c[0] /= factor;
		c[1] /= factor;
		c[2] /= factor;
		return *this;
	}

	/** division through a factor
	* @param factor to multiply
	* @return product
	*/
	Matrix3x3<T> operator/(const T& factor) const
	{
		return Matrix3x3<T>(*this) /= factor;
	}
	

	/** negates the Matrix3x3 
	* @return neg
	*/
	Matrix3x3<T> operator-() const
	{
		return Matrix3x3<T>(Vector3<T>(), Vector3<T>(), Vector3<T>()) -= *this;
	}

	/** comparison of another Matrix3x3 to this one
	* @param other Matrix3x3
	* @return equality
	*/
	bool operator==(const Matrix3x3<T>& other) const
	{
		return (c[0] == other.c[0] && 
				c[1] == other.c[1] && 
				c[2] == other.c[2]
		);
	}

	/** comparison of another Matrix3x3 to this one
	* @param other Matrix3x3
	* @return inequality
	*/
	bool operator!=(const Matrix3x3<T>& other) const
	{
		return !(*this == other);
	}


	/**
    * Calculation of the determinant of this matrix.
    * @return The determinant.
    */
	T det() const 
	{ 
		return 
		c[0].x * (c[1].y * c[2].z - c[1].z * c[2].y) +
		c[0].y * (c[1].z * c[2].x - c[1].x * c[2].z) +
		c[0].z * (c[1].x * c[2].y - c[1].y * c[2].x);
	}


	/**
	* Calculate the adjoint of this matrix.
	* @return the adjoint matrix.
	*/
	Matrix3x3<T> adjoint() const
	{
		return Matrix3x3<T>(
			Vector3<T>(
				det2(c[1].y, c[2].y, c[1].z, c[2].z), 
				det2(c[2].x, c[1].x, c[2].z, c[1].z), 
				det2(c[1].x, c[2].x, c[1].y, c[2].y)
			),
			Vector3<T>(
				det2(c[2].y, c[0].y, c[2].z, c[0].z), 
				det2(c[0].x, c[2].x, c[0].z, c[2].z), 
				det2(c[2].x, c[0].x, c[2].y, c[0].y)
			),
			Vector3<T>(
				det2(c[0].y, c[1].y, c[0].z, c[1].z), 
				det2(c[1].x, c[0].x, c[1].z, c[0].z), 
				det2(c[0].x, c[1].x, c[0].y, c[1].y)      
			)
		);  
	} 

	/**
	* Calculate determinant of 2x2 Submatrix  
	* | a b |
	* | c d |
	* @return  determinant.
	*/
	static T det2(T a, T b, T c, T d)
	{
		return a*d - b*c;
	}

	/**
	* Calculate the inverse of this matrix.
	* @return The inverse matrix
	*/
	Matrix3x3<T> invert() const
	{
		return adjoint().transpose() / det();
	}
};

#endif

