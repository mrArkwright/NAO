#ifndef __KinematicMatrix_h__
#define __KinematicMatrix_h__

#include "../Math/Vector3.h"
#include "../Math/RotationMatrix.h"
#include <sstream>

using namespace std;
/// Representation of Kinematic Information
/**
 * This class represents a KinematicMatrix
 * A KinematicMatrix is represented by a 3x3 RotationMatrix (rotM) and a Vector3 (posV)\n
 * \f{align*}{
 * \begin{bmatrix}
 *  rotM & posV \\
 *  0 &   1 \end{bmatrix}
 * \f}
 * The last row in a kinematic Matrix is always [ 0 0 0 1]
 * normally a KinematicMatrix should be of size 4x4, but because of the last row
 * the Matrix is only represented by a RotationMatrix and a PositionVector
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class KinematicMatrix 
{
public:
	/**
	 * The RotationMatrix 
	 */
	RotationMatrix rotM;

	/**
	 * The position vector
	 */
	Vector3<float> posV;

	/** default constructor (creates Identity Matrix) */
	KinematicMatrix()
	{
		rotM = RotationMatrix();
		posV = Vector3<float>();
	}

	/** constructor with initialization of the RotationMatrix 
	* @param rm the RotationMatrix
	*/
	KinematicMatrix(const RotationMatrix& rm)
	{
		rotM = rm;
		posV = Vector3<float>();
	}

	/** constructor with initialization of the position-vector
	* @param p the position-vector
	*/
	KinematicMatrix(const Vector3<float>& p)
	{
		rotM = RotationMatrix();
		posV = p;
	}

	/** constructor with initialization of the RotationMatrix and the position-vector
	* @param rm the RotationMatrix
	* @param p the position-vector
	*/
	KinematicMatrix(const RotationMatrix& rm, const Vector3<float>& p)
	{
		rotM = rm;
		posV = p;
	}

	/** copy constructor 
	* @param other the other KinematicMatrix
	*/
	KinematicMatrix(const KinematicMatrix& other)
	{
		*this = other;
	}

	/** inverts the KinematicMatrix
	* Note that because of the special structure, the inverse can be 
	* calculated by 
	* inv = |	inv(rotM)		-inv(RotM) * posV	|
	*		|		   0					    1	|
	*
	*/
	KinematicMatrix invert() const
	{
		RotationMatrix invRot = rotM.invert();
		Vector3<float> invPos = -invRot * posV;

		return KinematicMatrix(invRot, invPos);
	}

	/** creates a KinematicMatrix which represents a 
	* rotation about the x-axis
	* @param alpha Angle of rotation
	*/
	static KinematicMatrix rotX(const float& alpha)
	{
		return KinematicMatrix(RotationMatrix::rotX(alpha), Vector3<float>());
	}

	/** creates a KinematicMatrix which represents a 
	* rotation about the y-axis
	* @param alpha Angle of rotation
	*/
	static KinematicMatrix rotY(const float& alpha)
	{
		return KinematicMatrix(RotationMatrix::rotY(alpha), Vector3<float>());
	}

	/** creates a KinematicMatrix which represents a 
	* rotation about the z-axis
	* @param alpha Angle of rotation
	*/
	static KinematicMatrix rotZ(const float& alpha)
	{
		return KinematicMatrix(RotationMatrix::rotZ(alpha), Vector3<float>());
	}

	/** creates a KinematicMatrix which represents a 
	* translation along the x-axis
	* @param distance Distance of translation
	*/
	static KinematicMatrix transX(const float& distance)
	{
		return KinematicMatrix(RotationMatrix(), Vector3<float>(distance, 0, 0) );
	}

	/** creates a KinematicMatrix which represents a 
	* translation along the y-axis
	* @param distance Distance of translation
	*/
	static KinematicMatrix transY(const float& distance)
	{
		return KinematicMatrix(RotationMatrix(), Vector3<float>(0, distance, 0) );
	}

	/** creates a KinematicMatrix which represents a 
	* translation along the z-axis
	* @param distance Distance of translation
	*/
	static KinematicMatrix transZ(const float& distance)
	{
		return KinematicMatrix(RotationMatrix(), Vector3<float>(0, 0, distance) );
	}

	/** adds a KinematicMatrix to this one 
	* @param other other KinematicMatrix
	* @return sum
	*/
	KinematicMatrix& operator+=(const KinematicMatrix& other)
	{
		rotM += other.rotM;
		posV += other.posV;
		return *this;
	}

	/** adds a KinematicMatrix to another one
	* @param other other KinematicMatrix
	* @return sum
	*/
	KinematicMatrix& operator+(const KinematicMatrix& other) const
	{
		return KinematicMatrix(*this) += other;
	}

	/** subtracts a KinematicMatrix from this one 
	* @param other other KinematicMatrix
	* @return difference
	*/
	KinematicMatrix& operator-=(const KinematicMatrix& other)
	{
		rotM -= other.rotM;
		posV -= other.posV;
		return *this;
	}

	/** subtracts a KinematicMatrix from another one
	* @param other other KinematicMatrix
	* @return difference
	*/
	KinematicMatrix& operator-(const KinematicMatrix& other) const
	{
		return KinematicMatrix(*this) -= other;
	}

	/** multiplies a KinematicMatrix to this one 
	* @param other other KinematicMatrix
	* @return product
	*/
	KinematicMatrix& operator*=(const KinematicMatrix& other)
	{
		posV += rotM * other.posV;
		rotM *= other.rotM;		
		return *this;
	}

	/** multiplies a KinematicMatrix to another one
	* @param other other KinematicMatrix
	* @return product
	*/
	KinematicMatrix operator*(const KinematicMatrix& other) const
	{
		return KinematicMatrix(*this) *= other;
	}

	

	/** comparison of another KinematicMatrix to this one
	* @param other KinematicMatrix
	* @return equality
	*/
	bool operator==(const KinematicMatrix& other) const
	{
		return (rotM == other.rotM && posV == other.posV );
	}

	/** comparison of another KinematicMatrix to this one
	* @param other KinematicMatrix
	* @return inequality
	*/
	bool operator!=(const KinematicMatrix& other) const
	{
		return !(*this == other);
	}

	/** multiplication with a Vector3
	* This kind of multiplication allows to transformate coordinates
	* from one space to another
	* be careful: it is not a normal multiplication because of the special structure of Kinematic Matrices
	* @param position in source space
	* @return transformated position
	*/
	Vector3<float> operator*(const Vector3<float>& position) const
	{
		return rotM * position + posV;
	}
	
	/**
	 * Information of Matrix elements in a string. 
	 * Helpful for logging.
	 */
	string toString()
	{
		ostringstream s;
		s << "Rotation: \n";
		s << "x11: " << rotM.c[0].x << ", ";
		s << "x12: " << rotM.c[1].x << ", ";
		s << "x13: " << rotM.c[2].x << "\n";
		s << "x21: " << rotM.c[0].y << ", ";
		s << "x22: " << rotM.c[1].y << ", ";
		s << "x23: " << rotM.c[2].y << "\n";
		s << "x31: " << rotM.c[0].z << ", ";
		s << "x32: " << rotM.c[0].z << ", ";
		s << "x33: " << rotM.c[0].z << "\n";
		s << "Position: \n";
		s << "x: " << posV.x << ", ";
		s << "y: " << posV.y << ", ";
		s << "z: " << posV.z;
		
		return s.str();
	}
};
#endif
