/*
 * @Author: your name
 * @Date: 2021-09-07 16:29:35
 * @LastEditTime: 2022-06-13 16:30:27
 * @LastEditors: luo
 * @Description: In User Settings Edit
 * @FilePath: /ws_catkin/src/odom_pkg/src/odom_node.cpp
 */

#include <ros/ros.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <mutex>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <angles/angles.h>


using namespace std;

ros::Publisher odom_pub_;


float wheel_base_;
float wheel_radius_;
std::string config_path_;

float nr_speed = 0;
float nl_speed = 0;

float left_v = 0;
float right_v = 0;
float odom_v = 0;

float left_w = 0;
float right_w = 0;
float odom_w = 0;

float delta_left_position_ = 0;
float delta_right_position_ = 0;

float delta_s = 0;
float delta_th = 0;
float x = 0.0;
float y = 0.0;
float th = 0.0;

bool is_first_ = true;

nav_msgs::Odometry odom_data_;

ros::Time current_time, last_time;

void WheelOdomCallBack(const sensor_msgs::JointStatePtr& odom_msg)
{
    current_time = odom_msg->header.stamp;

    if(is_first_)
    {
        last_time = current_time;

        x = 0.0;
        y = 0.0;
        th = 0.0;

        delta_left_position_ = 0;
        delta_right_position_ = 0;

        is_first_ = false;

        return;
    }

    nl_speed = odom_msg->velocity.at(0) / 60.0;
    nr_speed = odom_msg->velocity.at(1) / 60.0;

    // nl_speed = odom_msg->velocity.at(0);
    // nr_speed = odom_msg->velocity.at(1);


    right_w = nr_speed * M_PI * 2;
    left_w = nl_speed * M_PI * 2;

    right_v = nr_speed * wheel_radius_ * M_PI * 2;
    left_v = nl_speed * wheel_radius_ * M_PI * 2;

    std::cout<< "velo_left = " << left_v << std::endl;
    std::cout<< "velo_right = " << right_v << std::endl;

    odom_v =  (right_v + left_v) / 2;
    odom_w = ( right_v - left_v) / wheel_base_ ;


    float dt = (current_time - last_time).toSec();
    delta_right_position_ = right_v * dt;
    delta_left_position_ = left_v * dt;

    std::cout<< "delta_left_p = " << delta_left_position_ << std::endl;
    std::cout<< "delta_right_p = " << delta_right_position_ << std::endl;

    delta_s = (delta_right_position_ + delta_left_position_) / 2.0;
    delta_th = odom_w * dt;

     
    last_time = current_time;
    
    x += delta_s * cos(delta_th + th);
    y += delta_s * sin(delta_th + th);
    th += delta_th;

    th = angles::normalize_angle(th);

    // if(th > M_PI)
    // {
    //     th -= (M_PI * 2);
    // }else if(th < -M_PI) 
    // {
    //     th += (M_PI * 2);
    // }

 
    std::cout<< "time = " << current_time << std::endl;
    std::cout<< "x = " << x << std::endl;
    std::cout<< "y = " << y << std::endl;
    std::cout<< "th = " << angles::to_degrees(th) << std::endl;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    odom_data_.header.stamp = current_time;
    odom_data_.header.frame_id = "odom";
    odom_data_.child_frame_id = "base_footprint";

    odom_data_.pose.pose.position.x = x;
    odom_data_.pose.pose.position.y = y;
    odom_data_.pose.pose.position.z = 0.0;
    odom_data_.pose.pose.orientation = odom_quat;

    odom_pub_.publish(odom_data_);

}



int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odom_node");

    ros::NodeHandle nh;

    std::string config_path_ = "/home/luo/carto_ws/catkin_works/src/odom_pkg/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path_);
    wheel_base_ = config_node["wheel_base"].as<float>();
    wheel_radius_ = config_node["wheel_radius"].as<float>();

    ros::Subscriber odom_sub = nh.subscribe("/motor_velocity", 2, WheelOdomCallBack);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>("/encoder_odom", 2);

    ros::spin();

    return 0;
}