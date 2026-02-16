#include "KalmanFilter.h"

KalmanFilter::KalmanFilter(double dt, double sigma)
{
	A << 1, 0, dt, 0,
		0, 1, 0, dt,
		0, 0, 1, 0,
		0, 0, 0, 1;
	H << 1, 0, 0, 0,
		0, 1, 0, 0;
	Q = Matrix4d::Identity() * q_scale;
	R = Matrix2d::Identity() * (sigma * sigma);
	P = Matrix4d::Identity() * p0_scale;
	x.setZero();
}

void KalmanFilter::init(double mx, double my)
{
	x << mx, my, 0.0, 0.0;
}

void KalmanFilter::Predict()
{
	x = A * x;
	P = A * P * A.transpose() + Q;
}

void KalmanFilter::Update(Vector2d z)
{
	Vector2d y = z;
	Vector2d nu = y - H * x;
	Matrix2d S = H * P * H.transpose() + R;
	Matrix <double, 4, 2> K = P * H.transpose() * S.ldlt().solve(Matrix2d::Identity());
	x = x + K * nu;
	P = (Matrix4d::Identity() - K * H) * P;
}

Vector4d KalmanFilter::State()
{
	return x;
}
