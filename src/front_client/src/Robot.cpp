//
// Created by lbw on 25-1-16.
//

#include "Robot.h"

Robot::Robot(): Node("robot"), v_yaw_(1), yaw_(0),r_(0.4)
{
    height_ = declare_parameter("height", 0.2f);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(1),
        [this]() { return timer_callback(); });

    robot_pub_ = create_publisher<RobotInfo>(
        "front_client/robot_status",
        10);

    markers_pub_ = create_publisher<MarkerArray>(
        "front_client/markers",
        10);

    center_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    camera_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    timestamp_ = this->get_clock()->now();
    initRobot();
    initMarkers();
}

void Robot::timer_callback()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera";
    t.child_frame_id = "center";
    t.transform.translation.x = center_.x;
    t.transform.translation.y = center_.y;
    t.transform.translation.z = center_.z;
    center_broadcaster_->sendTransform(t);
    publishMarkers();
    publishRobotInfo();

}

void Robot::initRobot()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution dis(0.0, 3.0); // 定义一个范围在0.0到1.0的均匀分布

    //定义中心点的坐标系
    center_.x = dis(gen);
    center_.y = dis(gen);
    center_.z = height_;
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "camera";
    t.child_frame_id = "center";
    t.transform.translation.x = center_.x;
    t.transform.translation.y = center_.y;
    t.transform.translation.z = center_.z;
    center_broadcaster_->sendTransform(t);

    //定义相机坐标
    geometry_msgs::msg::TransformStamped tt;
    tt.header.stamp = this->get_clock()->now();
    tt.child_frame_id = "camera";
    tt.transform.translation.x = tt.transform.translation.y = tt.transform.translation.z = 0.0;
    tt.transform.rotation.x = 0.0;
    tt.transform.rotation.y = 0.0;
    tt.transform.rotation.z = 0.0;
    tt.transform.rotation.w = 1.0;
    camera_broadcaster_->sendTransform(tt);
}

void Robot::initMarkers()
{
    center_marker_.header.frame_id = "center";
    center_marker_.ns = "center";
    center_marker_.id = 0;
    center_marker_.header.stamp = this->get_clock()->now();
    center_marker_.type = Marker::SPHERE;
    center_marker_.scale.x = center_marker_.scale.y = center_marker_.scale.z = 0.2;
    center_marker_.color.a = 1.0;
    center_marker_.color.g = 1.0;

    armor_marker_.header.frame_id = "center";
    armor_marker_.ns = "armor";
    armor_marker_.header.stamp = this->get_clock()->now();
    armor_marker_.type = Marker::CUBE;
    armor_marker_.color.g = 1.0;
    armor_marker_.color.b = 1.0;
    armor_marker_.scale.x = 0.02;
    armor_marker_.scale.y = 0.4;
    armor_marker_.scale.z = 0.2;
}

void Robot::publishMarkers()
{
    MarkerArray marker_array;


    center_marker_.header.stamp = this->get_clock()->now();
    center_marker_.action = Marker::ADD;
    marker_array.markers.push_back(center_marker_);

    auto T = this->get_clock()->now() - timestamp_;
    timestamp_ = this->get_clock()->now();
    for (int i = 0; i < 4; i++)
    {
        armor_marker_.action = Marker::ADD;
        yaw_ += v_yaw_*T.seconds();
        double tmp_yaw = yaw_ + i*M_PI/2;
        armor_marker_.pose.position.x = r_*cos(tmp_yaw);
        armor_marker_.pose.position.y = r_*sin(tmp_yaw);

        tf2::Quaternion q;
        q.setRPY(0,-0.2,tmp_yaw);
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
    markers_pub_->publish(marker_array);

}

void Robot::publishRobotInfo()
{
    RobotInfo msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "center";
    msg.center = center_;
    for (int i = 0; i < 4; i++)
    {
        double tmp_yaw = yaw_ + i*M_PI/2;

        tf2::Quaternion q;
        q.setRPY(0,-0.2,tmp_yaw);

        Armor armor;
        armor.pose.orientation.x = q.getX();
        armor.pose.orientation.y = q.getY();
        armor.pose.orientation.z = q.getZ();
        armor.pose.orientation.w = q.getW();
        armor.number = "3";

        msg.armors.push_back(armor);
    }

    robot_pub_->publish(msg);

}
