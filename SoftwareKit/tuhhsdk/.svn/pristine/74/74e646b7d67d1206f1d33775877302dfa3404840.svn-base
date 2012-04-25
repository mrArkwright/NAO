

#ifndef __KalmanFilter_h__
#define __KalmanFilter_h__

#include "../Math/Vector2.h"
#include "../Math/Matrix2x2.h"

using namespace std;

/// Implementation of a Kalman Filter
/**
 * This class realizes a KalmanFilter
 * @author <a href="mailto:stefan.kaufmann@tu-harburg.de">Stefan Kaufmann</a>
 */
class KalmanFilter
{
public:
	/** default constructor */
	KalmanFilter();

	/** constructor
	 * @param A The state matrix
	 * @param b The input Vector
	 * @param c The output Vector
	 * @param x The initial state Vector
	 * @param P Initial Guess on update Matrix
	 * @param Q Process noise covariance
	 * @param R Measurenment noise covariance
	 */
	KalmanFilter(const Matrix2x2<float>& A, const Vector2<float>& b, const Vector2<float>& c,
		const Vector2<float>& x, const Matrix2x2<float>& P, const Matrix2x2<float>& Q, const float& R);

	/**
	 * prediction with measurenment update
	 * @param u The input for the state space model
	 * @param measure The current measurenment
	 * @return The estimated state
	 */
	Vector2<float> predict(const float& u, const float & measure);

	/**
	 * set the covariances 
	 * @param Q process noise covariance
	 * @param R measurenment noise covariance
	 */
	void setCovariances(const Matrix2x2<float>& Q, const float&R);

	/**
	 * get Kalman gain
	 * @return The Kalman gain Vector
	 */
	Vector2<float> predictGain();
	


private:
	Vector2<float> K;
	Matrix2x2<float> A;
	Vector2<float> b;
	Vector2<float> c;
	Vector2<float> x;	
	Matrix2x2<float> P;
	Matrix2x2<float> Q;
	
	float R;
	

};
#endif