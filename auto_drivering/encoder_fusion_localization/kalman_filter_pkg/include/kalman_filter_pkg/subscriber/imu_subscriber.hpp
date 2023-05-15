/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:06:09
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:49:57
 */
#pragma once

#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"

namespace KalmanFilter
{
class IMUSubscriber
{
public:
  IMUSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  ~IMUSubscriber();
  void ParseData(std::deque<IMUData> &deque_imu_data);

private:
  void msg_callback(const sensor_msgs::ImuConstPtr &imu_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<IMUData> new_imu_data_;
};
} 