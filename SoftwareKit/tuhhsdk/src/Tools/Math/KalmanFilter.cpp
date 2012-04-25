#include "KalmanFilter.h"

KalmanFilter::KalmanFilter()
{
	A = Matrix2x2<float>();
	b = Vector2<float>(1,0);
	c = Vector2<float>(1,0);
	x = Vector2<float>();
	R = 1;
	Q = Matrix2x2<float>();
	P = Matrix2x2<float>();
}

KalmanFilter::KalmanFilter(const Matrix2x2<float> &A, const Vector2<float> &b, const Vector2<float> &c, const Vector2<float>& x, const Matrix2x2<float> &P, const Matrix2x2<float> &Q, const float &R):
A(A), b(b), c(c), x(x), P(P), Q(Q), R(R){};



void KalmanFilter::setCovariances(const Matrix2x2<float>& Q, const float& R)
{
	(*this).Q = Q;
	(*this).R = R;
}

Vector2<float> KalmanFilter::predict(const float& u, const float& measure)
{
	// predict next state
	x = A*x + b*u;
	
	// calculate update matrix
	P = A*P*A.transpose() + Q;

	// calculate Kalman gain
	K = P*c / (P.mulLeft(c) * c + R);

	// correct state
	x = x + K * (measure - c*x);

	// update of P
	P = P - Matrix2x2<float>(K*c.x,K*c.y)*P;
	
	return x;

}

Vector2<float> KalmanFilter::predictGain()
{
	// calculate update matrix
	P = A*P*A.transpose() + Q;

	// calculate Kalman gain
	K = P*c / (P.mulLeft(c) * c + R);

	// update of P
	P = P - Matrix2x2<float>(K*c.x,K*c.y)*P;

	return K;
}