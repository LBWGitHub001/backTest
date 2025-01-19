//
// Created by lbw on 25-1-16.
//

#ifndef ROTATION_H
#define ROTATION_H
//std
#include<memory>
#include<functional>
#include<Eigen/Dense>
//ros
#include<rclcpp/rclcpp.hpp>
//project
#include"Kalman.h"
#include"interfaces/srv/kalman1.hpp"

class Rotation {
using Kalman1 = interfaces::srv::Kalman1;
public:
    Rotation();
    ~Rotation();

    void predict(const std::shared_ptr<Kalman1::Request> request,
          std::shared_ptr<Kalman1::Response>      response);

    inline void setQ();

private:


    std::unique_ptr<KalmanFilter<3,1>> filter_;


    static constexpr int StateSize_ = 3;
    static constexpr int MeasurementSize_ = 1;
    double dt_;

    std::function<Eigen::MatrixXd(void)> A;
    Eigen::MatrixXd H;
    Eigen::MatrixXd P;
};



#endif //ROTATION_H
