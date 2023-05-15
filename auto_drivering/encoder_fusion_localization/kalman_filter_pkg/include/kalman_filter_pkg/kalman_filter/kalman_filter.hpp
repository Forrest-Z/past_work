/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-18 19:38:45
 * @LastEditors: luo
 * @LastEditTime: 2021-11-26 15:36:42
 */
#pragma once

#include <ros/ros.h>
#include <iostream>

#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/sensor_data/pose_data.hpp"

#include "kalman_filter_pkg/models/cloud_filter/cloud_filter_interface.hpp"
#include "kalman_filter_pkg/models/cloud_filter/box_filter.hpp"
#include "kalman_filter_pkg/models/cloud_filter/voxel_filter.hpp"

#include "kalman_filter_pkg/models/registration/registration_interface.hpp"
#include "kalman_filter_pkg/models/registration/registration_icp.hpp"
#include "kalman_filter_pkg/models/registration/registration_ndt.hpp"


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


class KalmanFilterProcess
{
public:
    KalmanFilterProcess();
    ~KalmanFilterProcess();

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool Predict(const std::deque<IMUData>& deque_imu_data);
    bool Correct();
private:
    
    double position_noise_ = 0.0;
    double gyro_noise_ = 0.0;
    double acc_noise_ = 0.0;


    bool has_inited_ = false;
    State state_;
    ErrorState error_state_;
    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

};

}