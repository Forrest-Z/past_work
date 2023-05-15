/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-09 16:53:39
 * @LastEditors: luo
 * @LastEditTime: 2021-12-27 10:40:14
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <mutex>
#include <deque>
#include <yaml-cpp/yaml.h>



#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/gnss_subscriber.hpp"
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"
#include "kalman_filter_pkg/publisher/cloud_publisher.hpp"
#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"

#include "kalman_filter_pkg/matching/matching.hpp"
namespace KalmanFilter
{
class MatchingFlow
{
public:
    MatchingFlow(ros::NodeHandle& nh);
    ~MatchingFlow();

    bool Run();
    bool ReadData();
    bool HasData();
    bool ValidData();

    bool UpdateMatching();

    bool PublishData();

private:
    ros::NodeHandle nh_;

    std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
    std::deque<CloudData> cloud_data_buff_;

    std::shared_ptr<OdometrySubscriber> gnss_sub_ptr_;
    std::deque<PoseData> gnss_data_buff_;

    std::shared_ptr<CloudPublisher> local_map_pub_ptr_;
    std::deque<CloudData> local_map_data_buff_;

    std::shared_ptr<CloudPublisher> global_map_pub_ptr_;
    std::deque<CloudData> global_map_data_buff_;

    std::shared_ptr<OdometryPublisher> laser_odom_pub_ptr_;
    
    PoseData cur_gnss_;
    CloudData cur_cloud_;
    CloudData last_cloud_;


    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f cur_pose_ = Eigen::Matrix4f::Identity();

    std::shared_ptr<Matching> matching_ptr_;

};
 
}