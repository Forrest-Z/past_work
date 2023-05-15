/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:10:40
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 18:30:19
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>

#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// #include "fusion_localization/sensor_data/cloud_data.hpp"
// #include "fusion_localization/sensor_data/imu_data.hpp"
// #include "fusion_localization/sensor_data/gnss_data.hpp"
#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/gnss_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"
#include "kalman_filter_pkg/tf_listener/tf_listener.hpp"
#include "kalman_filter_pkg/front_end/front_end.hpp"
#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"

namespace KalmanFilter
{
    
class FrontEndFlow
{
public:
    FrontEndFlow(ros::NodeHandle& nh);
    ~FrontEndFlow();

    bool Run();
    bool ReadData();
    bool InitCalibration();
    bool InitGNSS();
    bool HasData();
    bool ValidData();
    bool UpdateGNSSOdometry();
    bool UpdateLaserOdometry();
    bool PublishData();

private:
    ros::NodeHandle nh_;
    
    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::shared_ptr<GNSSSubscriber> gnss_sub_ptr_;
    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
    std::shared_ptr<TFListener> lidar_to_imu_ptr_;
    std::shared_ptr<OdometryPublisher> gnss_pub_ptr_;
    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;

    std::shared_ptr<FrontEnd> front_end_ptr_;
    

    std::deque<CloudData> cloud_data_buff_;
    std::deque<IMUData> imu_data_buff_;
    std::deque<GNSSData> gnss_data_buff_;
    std::deque<PoseData> odom_data_buff_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    GNSSData current_gnss_data_;
    PoseData current_odom_data_;

    CloudData::CloudPointTPtr local_map_ptr_;
    CloudData::CloudPointTPtr global_map_ptr_;
    CloudData::CloudPointTPtr current_scan_ptr_;


    Eigen::Matrix4f lidar_to_imu_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f gnss_odometry_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f laser_odometry_ = Eigen::Matrix4f::Identity();



};

}