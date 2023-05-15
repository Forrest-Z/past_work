/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-30 14:38:48
 * @LastEditors: luo
 * @LastEditTime: 2021-12-30 19:11:37
 */

#pragma once

#include <iostream>
#include <mutex>
#include <vector>
#include <queue>
#include <map>
#include <array>
#include <string>
#include <thread>

//gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include <ros/ros.h>
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"

using gtsam::symbol_shorthand::B; //Bias(ax, ay, az, gx, gy, gz)
using gtsam::symbol_shorthand::V; //Vel(xdot, ydot, zdot)
using gtsam::symbol_shorthand::X; //Pose3(x, y, z, r, p, y)

namespace KalmanFilter
{
class GtsamOptimizer
{
public:
    GtsamOptimizer(ros::NodeHandle& nh);
    ~GtsamOptimizer();

    void Run();

private:
    void ImuCallBack(const sensor_msgs::ImuConstPtr& imu_raw);
    void PriorOdometryCallBack(const nav_msgs::OdometryConstPtr& odom_msg);

    bool HasData();
    bool ValidData();
    bool UpdatePreintegration();
    bool PublishData();
    bool FailureDetection(const gtsam::Vector3& vel_cur, const gtsam::imuBias::ConstantBias& bias_cur);

    void ResetOptimization();
    void ResetParams();

    void Command();


private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_imu_;
    ros::Subscriber sub_prior_odometry_;

    ros::Publisher pub_imu_odometry_;

    std::shared_ptr<OdometryPublisher> pub_li_odom_ptr_;

    std::mutex mutex_;

    std::deque<sensor_msgs::ImuConstPtr> imu_opt_buffer_;
    std::deque<sensor_msgs::ImuConstPtr> imu_odom_buffer_;
    std::deque<nav_msgs::OdometryConstPtr> prior_odom_buffer_;

    nav_msgs::OdometryConstPtr current_prior_odom_pose_;
    double cur_prior_odom_time_;

    bool lio_opt_initialized_;

    Eigen::Matrix4f li_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Vector3f li_velocity_ = Eigen::Vector3f::Zero();

    gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_;      //先验位置噪声
    gtsam::noiseModel::Diagonal::shared_ptr prior_vel_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_bias_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr correction_noise_2_;
    gtsam::Vector noise_model_between_bias_;

    gtsam::PreintegratedImuMeasurements *imu_intrgrator_opt_;
    gtsam::PreintegratedImuMeasurements *imu_intrgrator_imu_;


    gtsam::Pose3 prev_pose_;                    //上一时刻估计的位姿信息
    gtsam::Vector3 prev_vel_;                    //上一时刻imu的速度信息
    gtsam::NavState prev_state_;
    gtsam::imuBias::ConstantBias prev_bias_;

    gtsam::NavState prev_state_odom_;
    gtsam::imuBias::ConstantBias prev_bias_odom_;

    //imu
    float imu_acc_noise_;
    float imu_gyro_noise_;
    float imu_acc_bias_N_;
    float imu_gyro_bias_N_;
    float imu_gravity_;

    bool done_first_opt_ = false;
    double last_imu_time_imu_ = -1;
    double last_imu_time_opt_ = -1;

    gtsam::ISAM2 isam_li_;
    gtsam::NonlinearFactorGraph graph_factor_li_;
    gtsam::Values initial_estimate_valus_li_;
    gtsam::Values isam_current_estimate_li_;

    const double delta_t_ = 0;

    int key = 1;

    Eigen::Matrix4d extrinsic_matrix_;
    gtsam::Pose3 imu_to_lidar_;
    gtsam::Pose3 lidar_to_imu_;

    std::vector<std::pair<double, std::array<float, 7>>> odom_li_;

}; 
}