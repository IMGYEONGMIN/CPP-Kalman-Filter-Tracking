#pragma once
#include<Eigen/Dense>
using namespace Eigen;
class KalmanFilter
{
private:
    Matrix4d A;
    Matrix4d Q;
    Matrix2d R;
    Matrix4d P;
    Vector4d x;
    Matrix<double, 2, 4> H;
    const double q_scale = 0.01;
    const double p0_scale = 10.0;

public:
    KalmanFilter(double dt, double sigma);
    void init(double mx, double my);
    void Predict();
    void Update(Vector2d z);
    Vector4d State();
};

