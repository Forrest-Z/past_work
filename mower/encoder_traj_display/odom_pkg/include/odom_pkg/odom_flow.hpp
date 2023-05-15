/*
 * @Author: your name
 * @Date: 2021-09-15 14:22:08
 * @LastEditTime: 2022-05-18 15:36:05
 * @LastEditors: luo
 * @Description: In User Settings Edit
 * @FilePath: /ws_catkin/src/odom_pkg/include/odom_pkg/odom_flow.hpp
 */
#pragma once

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

const float PI = 3.14;
const float PI2 = 6.28;


class WheelOdomMsg
{
public:
    WheelOdomMsg(ros::NodeHandle& nh);
    ~WheelOdomMsg();

    void WheelOdomCallBack(const sensor_msgs::JointState& odom_msg);

    void Process();

    bool PubOdom(float v, float w, float r);

private:
    int num_;
    float wheel_base_;
    float wheel_radius_;
    std::string config_path_;

    bool is_fisrst_{true};
    float nr_speed = 0;
    float nl_speed = 0;
    float left_v = 0;
    float right_v = 0;
    float odom_v = 0;

    float left_w = 0;
    float right_w = 0;
    float odom_w = 0;

    float delta_s = 0;
    float delta_th = 0;
    float x = 0.0;
    float y = 0.0;
    float th = 0.0;

    float x1 = 0.0;
    float y1 = 0.0;
    float th1 = 0.0;
    float th2 = 0.0;
    float th3 = 0.0;
    float delta_yaw = 0.0;

    sensor_msgs::JointState sensor_odom_ptr;
    sensor_msgs::Imu current_imu_data_;
    sensor_msgs::Imu last_imu_data_;
    sensor_msgs::Imu odom_current_imu_data_;
    float eqtc_FL1_;
    float eqtc_FL2_;
    float eqtc_FR1_;
    float eqtc_FR2_;
    
    float left_last_position_ = 0;
    float right_last_position_ = 0;
    float left_current_position_ = 0;
    float right_current_position_ = 0;
    float delta_left_position_ = 0;
    float delta_right_position_ = 0;

    ros::Time current_time, last_time;

    nav_msgs::Odometry odom_data_;
    nav_msgs::Odometry odom2_data_;
    std::mutex buff_mutex_;

public:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;
    ros::Publisher odom_pub_;
    ros::Publisher odom2_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher odom_2d_pub_;

    tf::TransformBroadcaster odom_broadcaster_;
    tf::TransformBroadcaster odom_broadcaster2_;

};