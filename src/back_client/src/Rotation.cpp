//
// Created by lbw on 25-1-16.
//

#include "Rotation.h"

Rotation::Rotation()
{

    H.resize(MeasurementSize_, StateSize_);
    H << 1, 0, 0;
    A = [this]() {
        Eigen::MatrixXd A_(StateSize_, StateSize_);
        A_ << 1, dt_, 0.5 * dt_ * dt_,
                0, 1, dt_,
                0, 0, 1;
        return A_;
    };
    auto Q = [this]() {
        Eigen::MatrixXd U(MeasurementSize_, MeasurementSize_);
        Eigen::MatrixXd G(StateSize_, MeasurementSize_);
        U << s2qyaw;
        G << dt_, dt_, dt_;
        Eigen::MatrixXd ans(MeasurementSize_,MeasurementSize_);

        ans = G * U * G.transpose();
        return ans;
    };
    auto R = [this](const Eigen::MatrixXd &z) {
        Eigen::MatrixXd R_(MeasurementSize_, MeasurementSize_);
        R_ << r_yaw_ * std::abs(z[0]);
        return R_;
    };
    P = Eigen::MatrixXd::Identity(StateSize_, StateSize_);

    filter_ = std::make_unique<KalmanFilter<3,1>>(A(), H, P, Q, R);


}

void Rotation::predict(const std::shared_ptr<Kalman1::Request> request, std::shared_ptr<Kalman1::Response> response)
{
}
