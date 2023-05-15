/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:51:01
 * @LastEditors: luo
 * @LastEditTime: 2021-11-22 14:37:53
 */
#pragma once

#include <deque>
#include <mutex>
#include <thread>
#include <string>

#include <iostream>
#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

// #include "fusion_localization/sensor_data/posVel.h"
#include "/home/luo/work/filter_ws/devel/include/kalman_filter_pkg/PosVelMag.h"

#include "/home/luo/work/filter_ws/devel/include/kalman_filter_pkg/PosVel.h"
// #include "fusion_localization/PosVelMag.h"
// #include "PosVelMag.h"
#include "kalman_filter_pkg/sensor_data/pose_vel_data.hpp"

using namespace KalmanFilter;
using namespace std;

namespace KalmanFilter
{
class PosVelSubscriber
{
public:
    PosVelSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    ~PosVelSubscriber();

    void ParseData(std::deque<PosVelData>& pos_vel_data_buff);


private:
    void Msg_CallBack(const kalman_filter_pkg::PosVelConstPtr& pos_vel_msg_ptr);
    // void msg_callback(const fusion_localization::P)

private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<PosVelData> new_pos_vel_data_;

    std::mutex buff_mutex_;
};
}