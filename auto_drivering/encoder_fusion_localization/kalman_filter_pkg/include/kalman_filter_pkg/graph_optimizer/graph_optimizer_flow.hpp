/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:43:57
 * @LastEditors: luo
 * @LastEditTime: 2021-12-21 17:37:37
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"

#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"

#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"

#include "kalman_filter_pkg/graph_optimizer/graph_optimizer.hpp"

namespace KalmanFilter
{
class GraphOptimizerFlow
{
public:
    GraphOptimizerFlow(ros::NodeHandle& nh);
    ~GraphOptimizerFlow();

    bool Run();
    bool ReadData();
    bool ValidData();
    bool HasData();

    bool Update();
    bool UpdateIMUPreIntegration();

    
    bool PublishData();

private:
    ros::NodeHandle nh_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::deque<CloudData> cloud_deta_buff_;

    std::shared_ptr<IMUSubscriber> imu_raw_sub_ptr_;
    std::deque<IMUData> imu_raw_data_buff_;

    std::shared_ptr<IMUSubscriber> imu_sys_sub_ptr_;
    std::deque<IMUData> imu_sys_data_buff_;

    std::shared_ptr<OdometrySubscriber> lidar_odom_sub_ptr_;
    std::deque<PoseData> lidar_odom_data_buff_;

    std::shared_ptr<OdometryPublisher> fusion_odom_pub_ptr_;

    std::shared_ptr<GraphOptimizer> graph_optimizer_ptr_;

    CloudData current_cloud_data_;
    IMUData current_imu_data_;
    PoseData current_lidar_odom_data_;


};
    
} // namespace KalmanFilter
