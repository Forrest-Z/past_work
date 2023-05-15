/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-26 15:35:55
 * @LastEditors: luo
 * @LastEditTime: 2021-11-26 15:37:15
 */
#pragma once

#include <iostream>
#include <ros/ros.h>

namespace KalmanFilter
{
constexpr double kDegreeToRatian = M_PI / 180.0;
constexpr double kRatianToDegree = 180.0 / M_PI;

class State
{
public:

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Quaterniond pose;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;

    IMUData imu_data;
};

class ErrorState
{
public:
    Eigen::Matrix<double, 15, 1> x;
    Eigen::Matrix<double, 15, 15> p;

};


}