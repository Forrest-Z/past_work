/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-18 19:38:38
 * @LastEditors: luo
 * @LastEditTime: 2022-01-07 15:58:06
 */
#pragma once

#include <nav_msgs/Odometry.h>
#include <deque>

#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include "kalman_filter_pkg/matching/matching.hpp"
#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"

#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"

#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"
#include "kalman_filter_pkg/publisher/cloud_publisher.hpp"

#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"
#include "kalman_filter_pkg/subscriber/encoder_subsciber.hpp"

// #include "kalman_filter_pkg/kalman_filter/kalman_filter.hpp"

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

};

class ErrorState
{
public:
    Eigen::Matrix<double, 15, 1> x;
    Eigen::Matrix<double, 15, 15> p;

};

class KalmanFilterFlow
{
public:
    KalmanFilterFlow(ros::NodeHandle& nh);
    ~KalmanFilterFlow();

    bool Run();
    bool ReadData();
    bool HasData();
    bool SynceddData(bool inited);
    bool ValidData();

    bool InitCalibration();
    bool InitPose();
    bool SetInitPose();

    Eigen::Matrix4f GetPose();

    bool CalculateEncoderPosition(EncoderData& encoder_msg, IMUData& imu_msg, double dt);
    bool CalculateEncoderOdom(EncoderData& encoder_msg, IMUData& imu_msg, double dt);

    bool Filter();
    bool Predict();
    bool Correct();

    bool PredictProcess();
    bool CorrectProcess();
    bool CorrectFromEncoder();

    bool PublishData();

    Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d w;
        w << 0., -v(2), v(1), 
             v(2), 0., -v(0), 
             -v(1), v(0), 0.;

        return w;
    }

private:
    ros::NodeHandle nh_;

    std::shared_ptr<EncoderSubscriber> encoder_sub_ptr_;
    std::deque<EncoderData> encoder_data_buff_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::deque<CloudData> cloud_data_buff_;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff_;

    std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
    std::deque<PoseData> odom_data_buff_;

    std::shared_ptr<Matching> matching_ptr_;
    // std::shared_ptr<KalmanFilterProcess> kalman_filter_process_ptr_;

    std::shared_ptr<OdometryPublisher> encoder_odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> odom_publish_ptr_;
    std::shared_ptr<OdometryPublisher> matching_odom_publish_ptr_;

    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;

    EncoderData current_encoder_data_;
    std::deque<EncoderData> deque_encoder_data_;
    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    std::deque<IMUData> deque_imu_data_;
    PoseData current_odom_data_;

    double delta_encoder_x = 0.0;
    double delta_encoder_y = 0.0;
    Eigen::Vector3d encoder_position{0.0, 0.0, 0.0};

    double encoder_velo = 0.0;
    double position_x = 0.0;
    double position_y = 0.0;
    double pose_yaw = 0.0;

    bool has_inited_ = false;
    State state_;
    ErrorState error_state_;

    double position_noise_ = 0.0;
    double gyro_noise_ = 0.0;
    double acc_noise_ = 0.0;
    double gyro_bias_noise_ = 0.0;
    double acc_bias_noise_ = 0.0;
    double v_noise_ = 0.0;

    double laser_pose_noise_ = 0.0;
    double laser_position_noise_ = 0.0;


    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f encoder_to_imu_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f kalman_filter_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f matching_odom = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f encoder_odom = Eigen::Matrix4f::Identity();
    
};
}