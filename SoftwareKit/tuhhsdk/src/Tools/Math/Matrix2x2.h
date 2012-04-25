#ifndef __Matrix2x2_h__
#define __Matrix2x2_h__

#include "Vector2.h"

/// Represenatiotn of 2x2 Matrix
/**
 * This class represents a 2x2-matrix. \n
 * Inspired by <a href="http://www.b-human.de/file_download/32/bhuman10_coderelease.tar.bz2">BHuman Code-Release 2010</a> 
 *
 * modified and extended by <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 *
 */
template<class T = float> class Matrix2x2
{
public:
	/** Two Vector2 represent the Matrix */
	Vector2<T> c[2];

	/** default constructor, creates a 2x2 identity matrix*/
	Matrix2x2<T>()		
	{
		c[0] = Vector2<T>(1,0);
		c[1] = Vector2<T>(0,1);
	}

	/** copy-constructor */
	Matrix2x2<T>(const Matrix2x2<T>& other)
	{
		*this = other;
	}   

	/** constructor with elementwise initialization */
	Matrix2x2<T>(
		const T& m11, const T& m12,
		const T& m21, const T& m22)
	{
		c[0] = Vector2<T>(m11,m21);
		c[1] = Vector2<T>(m12,m22);		
	}

	/** constructor with columnwise initialization */
	Matrix2x2<T>(const Vector2<T>& c0, const Vector2<T>& c1) 
	{
		c[0]=c0;
		c[1]=c1;
	}

	/** returns the transposed Matrix 
	* @return transposed
	*/
	Matrix2x2<T> transpose() const
	{
		return Matrix2x2<T>(
			Vector2<T>(c[0].x, c[1].x),
			Vector2<T>(c[0].y, c[1].y));		
	}

	/** adds a Matrix2x2 to this one 
	* @param other other Matrix2x2
	* @return sum
	*/
	Matrix2x2<T>& operator+=(const Matrix2x2<T>& other)
	{
		c[0] += other.c[0];
		c[1] += other.c[1];
		return *this;
	}

	/** adds a Matrix2x2 to this one 
	* @param other other Matrix2x2
	* @return sum
	*/
	Matrix2x2<T> operator+(const Matrix2x2<T>& other) const
	{
		return Matrix2x2<T>(*this) += other;
	}

	/** subtracts a Matrix2x2 from this one 
	* @param other Matrix2x2 which shall be subtracted from this one (subtrahend)
	* @return difference
	*/
	Matrix2x2<T>& operator-=(const Matrix2x2<T>& other)
	{
		c[0] -= other.c[0];
		c[1] -= other.c[1];
		return *this;
	}

	/** subtracts a Matrix2x2 from this one 
	* @param other Matrix2x2 which shall be subtracted from this one (subtrahend)
	* @return difference
	*/
	Matrix2x2<T> operator-(const Matrix2x2<T>& other) const
	{
		return Matrix2x2<T>(*this) -= other;
	}

	/** multiplies this Matrix2x2 by a factor
	* @param factor
	* @return product
	*/
	Matrix2x2<T>& operator*=(const T& factor)
	{
		c[0] *= factor;
		c[1] *= factor;
		return *this;
	}

	/** multiplication by a factor
	* @param factor to multiply
	* @return product
	*/
	Matrix2x2<T> operator*(const T& factor) const
	{
		return Matrix2x2<T>(*this) *= factor;
	}

	/** left side multiplication
	 * @param vector The multiplication vector
	 * @return result of (vector' * matrix)
	 */
	Vector2<T> mulLeft(const Vector2<T>& vector) const
	{
		return Vector2<float>(vector.x*c[0].x + vector.y*c[0].y, vector.x*c[1].x + vector.y*c[1].y);
	}


	/** multiplies this Matrix2x2 by a vector
	* @param vector
	* @return product
	*/
	Vector2<T> operator*(const Vector2<T>& vector) const
	{
		return (c[0]*vector.x + c[1]*vector.y);
	}

	/** multiplication by another Matrix2x2
	* @param other Matrix2x2
	* @return product
	*/
	Matrix2x2<T> operator*(const Matrix2x2<T>& other) const
	{
		Vector2<T> v1 = *this*other.c[0];
		Vector2<T> v2 = *this*other.c[1];
		return Matrix2x2<T>(v1,v2);
		
	}	


	/** multiplication by another Matrix2x2
	* @param other Matrix2x2
	* @return product
	*/
	Matrix2x2<T>& operator*=(const Matrix2x2<T>& other)
	{
		return *this = *this * other;
	}


	/** divides a Matrix2x2 through a factor
	* @param factor
	* @return quotient
	*/
	Matrix2x2<T>& operator/=(const T& factor)
	{
		c[0] /= factor;
		c[1] /= factor;
		return *this;
	}

	/** division through a factor
	* @param factor to multiply
	* @return product
	*/
	Matrix2x2<T> operator/(const T& factor) const
	{
		return Matrix2x2<T>(*this) /= factor;
	}
	

	/** negates the Matrix2x2 
	* @return neg
	*/
	Matrix2x2<T> operator-() const
	{
		return Matrix2x2<T>(Vector2<T>(), Vector2<T>()) -= *this;
	}

	/** comparison of another Matrix2x2 to this one
	* @param other Matrix2x2
	* @return equality
	*/
	bool operator==(const Matrix2x2<T>& other) const
	{
		return (c[0] == other.c[0] && 
				c[1] == other.c[1] 				
		);
	}

	/** comparison of another Matrix2x2 to this one
	* @param other Matrix2x2
	* @return inequality
	*/
	bool operator!=(const Matrix2x2<T>& other) const
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
		c[0].x * c[1].y - c[1].x * c[0].y;
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
	Matrix2x2<T> invert() const
	{
		return Matrix2x2(c[1].y, -c[1].x, -c[0].y, c[0].x) / det();
	}
};

#endif

