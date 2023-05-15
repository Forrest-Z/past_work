/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:20:24
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 16:09:27
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>

#include <nav_msgs/Odometry.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/sensor_data/gnss_data.hpp"

#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/gnss_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/velocity_subscriber.hpp"

#include "kalman_filter_pkg/publisher/cloud_publisher.hpp"
#include "kalman_filter_pkg/publisher/imu_publisher.hpp"
#include "kalman_filter_pkg/publisher/pos_vel_publisher.hpp"
#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"
#include "kalman_filter_pkg/tf_listener/tf_listener.hpp"

namespace KalmanFilter
{
class DataPrestreatFlow
{
public:
    DataPrestreatFlow(ros::NodeHandle& nh);
    ~DataPrestreatFlow();

    bool Run();
    bool InitCalibration();
    bool InitGNSS();
    bool ReadData();
    bool HasData();
    bool ValiedData();
    bool PublishData();
    bool TransformData();

private:
    ros::NodeHandle nh_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> encoder_odom_sub_ptr_;
    

    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<PoseData> odom_data_buff_;
    std::deque<PoseData> encoder_odom_data_buff_;

    std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
    std::shared_ptr<IMUPublisher> imu_pub_ptr_;
    // std::shared_ptr<PosVelPublisher> pose_vel_pub_ptr_;
    std::shared_ptr<OdometryPublisher> odom_pub_ptr_;
    std::shared_ptr<OdometryPublisher> encoder_odom_pub_ptr_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    PoseData current_odom_data_;
    PoseData current_encoder_odom_data_;

    Eigen::Matrix4f odom_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f encoder_odom_pose_ = Eigen::Matrix4f::Identity();




};

}