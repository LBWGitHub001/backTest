//
// Created by lbw on 25-1-15.
//

#ifndef BACK_H
#define BACK_H
//std
#include<vector>
#include<random>
#include<Eigen/Dense>
//ros
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<tf2_ros/buffer.h>
#include<visualization_msgs/msg/marker.hpp>
#include<visualization_msgs/msg/marker_array.hpp>
//project
#include"interfaces/msg/armors.hpp"
#include"interfaces/msg/robot.hpp"
#include"interfaces/srv/kalman1.hpp"
#include"Translate.h"
#include"Rotation.h"

class Back :
public rclcpp::Node{
    using RobotInfo = interfaces::msg::Robot;
    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Point = geometry_msgs::msg::Point;

public:
    Back();
    ~Back() = default;

private:
    rclcpp::Subscription<RobotInfo>::SharedPtr robot_sub_;
    void robotCallback(const RobotInfo::SharedPtr msg);
    //坐标系发布
    std::unique_ptr<tf2_ros::TransformBroadcaster> predict_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    void timerCallback();

    //Markers
    rclcpp::Time timestamp_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
    Marker armor_marker_;
    Marker center_marker_;
    void initMarkers();

    //系统初始化
    void publishMarkers(const RobotInfo& robot);

    //参数
    RobotInfo robot_info_;
};



#endif //BACK_H
