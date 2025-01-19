//
// Created by lbw on 25-1-16.
//

#ifndef ROBOT_H
#define ROBOT_H
//std
#include<vector>
#include<random>

//ros
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/point.hpp>
#include<tf2_ros/transform_broadcaster.h>
#include<tf2_ros/static_transform_broadcaster.h>
#include<tf2_ros/buffer.h>
#include<visualization_msgs/msg/marker.hpp>
#include<visualization_msgs/msg/marker_array.hpp>
#include"interfaces/msg/armors.hpp"
#include"interfaces/msg/robot.hpp"

class Robot :
    public rclcpp::Node
{
    using RobotInfo = interfaces::msg::Robot;
    using Armor = interfaces::msg::Armor;
    using Armors = interfaces::msg::Armors;
    using Marker = visualization_msgs::msg::Marker;
    using MarkerArray = visualization_msgs::msg::MarkerArray;
    using Point = geometry_msgs::msg::Point;

public:
    Robot();
    ~Robot() = default;

private:
    //发布节点
    rclcpp::TimerBase::SharedPtr timer_;
    void timer_callback();
    rclcpp::Publisher<RobotInfo>::SharedPtr robot_pub_;
    //坐标系发布
    std::unique_ptr<tf2_ros::TransformBroadcaster> center_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> camera_broadcaster_;

    //Markers
    rclcpp::Time timestamp_;
    rclcpp::Publisher<MarkerArray>::SharedPtr markers_pub_;
    Marker armor_marker_;
    Marker center_marker_;
    void initMarkers();

    //系统初始化
    void initRobot();
    void publishMarkers();
    void publishRobotInfo();

    //参数
    double height_;
    Point center_;
    double yaw_;
    double v_yaw_;
    double r_;

};


#endif //ROBOT_H
