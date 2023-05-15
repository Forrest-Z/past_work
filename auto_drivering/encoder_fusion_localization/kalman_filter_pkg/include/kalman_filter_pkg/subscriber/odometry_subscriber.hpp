/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-08 09:38:47
 * @LastEditors: luo
 * @LastEditTime: 2021-11-20 19:20:49
 */
#pragma once

#include <iostream>
#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "kalman_filter_pkg/sensor_data/pose_data.hpp"

namespace KalmanFilter
{
class OdometrySubscriber
{
public:
    OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    ~OdometrySubscriber();

    void ParseData(std::deque<PoseData>& deque_pose_data);
    void ParseOdomData(std::deque<nav_msgs::Odometry>& deque_pose_data);

private:
    void Msg_CallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr);

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PoseData> new_pose_data_;
    std::deque<nav_msgs::Odometry> new_odom_data_;

    std::mutex buff_mutex_;
};
}