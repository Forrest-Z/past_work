/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-24 20:20:56
 * @LastEditors: luo
 * @LastEditTime: 2022-01-06 12:55:01
 */
#pragma once

#include <iostream>
#include <math.h>
#include <deque>
#include <mutex>

#include <Eigen/Dense>

// #include <sensor_msgs/PointCloud2.h>

#include "/home/luo/work/filter_ws/devel/include/eqyc_joy2can/eqyc_IMCU_msg.h"

namespace KalmanFilter
{
//两轮距离
constexpr double wheel_base = 1.19;
//车轮半径:  直径：0.755
constexpr double wheel_radius = 0.375;
//齿轮的齿数
constexpr double num = 80.0;

// double PI = 3.1415926;

class EncoderData
{
public:
    struct Velo_Left
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Velo_Right
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

public:
    double time = 0.0;

    std::deque<double> status_buff;
    // std::deque<double> right_status_buff;
    bool status = false;
    double vehicle_speed = 0.0;

    int control_mode_status = 0;
    double frequency_l1 = 0.0;
    double frequency_l2 = 0.0;

    double frequency_r1 = 0.0;
    double frequency_r2 = 0.0;

    double n_left = 0.0;
    double n_right = 0.0;

    double velo_left = 0.0;
    double velo_right = 0.0;

    double w_left = 0.0;
    double w_right = 0.0;

    double v_encoder = 0.0;
    double w_encoder = 0.0;
    double r_encoder = 0.0;

    Eigen::Vector3d encoder_velocity = Eigen::Vector3d::Zero();

public:
    bool CalculateNLeft();
    bool CalculateNRight();
    bool CalculateVelo();

    bool GetStatus();

    bool SyncData(std::deque<EncoderData> & UnsyncedData, std::deque<EncoderData> &SyncedData, double sync_time);
};
}