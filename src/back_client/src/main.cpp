//
// Created by lbw on 25-1-15.
//
#include <rclcpp/rclcpp.hpp>
#include "Back.h"
int main (int argc, char ** argv){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Back>());
    rclcpp::shutdown();
}