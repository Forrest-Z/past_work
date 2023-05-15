/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:40
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:40:13
 */
#pragma once


#include <deque>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"

namespace KalmanFilter
{
class CloudSubscriber
{
public:
  CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  ~CloudSubscriber();
  void ParseData(std::deque<CloudData> &deque_cloud_data);

private:
  void msg_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<CloudData> new_cloud_data_;
};
} // namespace lidar_localization
