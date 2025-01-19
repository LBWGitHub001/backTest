//
// Created by lbw on 25-1-15.
//

#include "Back.h"

Back::Back()
    : rclcpp::Node("back")
{
    robot_sub_ = this->create_subscription<RobotInfo>(
        "front_client/robot_status",
        10,
        [this](const RobotInfo::SharedPtr msg) { return robotCallback(msg); });

    markers_pub_ = this->create_publisher<MarkerArray>(
        "back_client/markers",
        10);

    predict_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() { return this->timerCallback(); });

    markers_pub_ = this->create_publisher<MarkerArray>(
        "back_client/markers",
        10);

    timestamp_ = this->get_clock()->now();
    initMarkers();
}

void Back::initMarkers()
{
    //中心
    center_marker_.header.frame_id = "prediction";
    center_marker_.ns = "center";
    center_marker_.id = 0;
    center_marker_.header.stamp = this->get_clock()->now();
    center_marker_.type = Marker::SPHERE;
    center_marker_.scale.x = center_marker_.scale.y = center_marker_.scale.z = 0.2;
    center_marker_.color.a = 1.0;
    center_marker_.color.g = 1.0;

    //装甲板
    armor_marker_.header.frame_id = "prediction";
    armor_marker_.ns = "armor";
    armor_marker_.header.stamp = this->get_clock()->now();
    armor_marker_.type = Marker::CUBE;
    armor_marker_.color.g = 1.0;
    armor_marker_.color.b = 1.0;
    armor_marker_.scale.x = 0.02;
    armor_marker_.scale.y = 0.4;
    armor_marker_.scale.z = 0.2;
}

void Back::publishMarkers(const RobotInfo& robot)
{/*
    MarkerArray marker_array;
    center_marker_.header.stamp = this->get_clock()->now();
    center_marker_.action = Marker::ADD;
    marker_array.markers.push_back(center_marker_);

    auto T = this->get_clock()->now() - timestamp_;
    timestamp_ = this->get_clock()->now();
    for (int i = 0; i < 4; i++)
    {
        armor_marker_.action = Marker::ADD;
        double tmp_yaw = yaw_ + i * M_PI / 2;
        armor_marker_.pose.position.x = robot.r * cos(tmp_yaw);
        armor_marker_.pose.position.y = robot.r * sin(tmp_yaw);

        tf2::Quaternion q;
        q.setRPY(0, -0.2, tmp_yaw);
        armor_marker_.pose.orientation.x = q.getX();
        armor_marker_.pose.orientation.y = q.getY();
        armor_marker_.pose.orientation.z = q.getZ();
        armor_marker_.pose.orientation.w = q.getW();
        armor_marker_.id = i;
        marker_array.markers.push_back(armor_marker_);

        Armor armor;
        armor.pose.orientation.x = q.getX();
        armor.pose.orientation.y = q.getY();
        armor.pose.orientation.z = q.getZ();
        armor.pose.orientation.w = q.getW();
        armor.number = "3";
    }
    markers_pub_->publish(marker_array);*/
}

void Back::timerCallback()
{
    auto now = this->get_clock()->now();
    auto T = now - timestamp_;
    timestamp_ = now;


    publishMarkers(robot_info_);
}

void Back::robotCallback(const RobotInfo::SharedPtr msg)
{
    //发布Predict参考系
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = "center";
    t.child_frame_id = "prediction";
    t.transform.translation.z = 1;
    predict_broadcaster_->sendTransform(t);

    //复制一份数据
    robot_info_ = *msg;
}

