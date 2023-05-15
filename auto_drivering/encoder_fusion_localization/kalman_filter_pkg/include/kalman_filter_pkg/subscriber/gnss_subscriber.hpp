/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:59
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:40:16
 */
#pragma once


#include <deque>
#include <ros/ros.h>
#include "sensor_msgs/NavSatFix.h"
#include "kalman_filter_pkg/sensor_data/gnss_data.hpp"

namespace KalmanFilter
{
class GNSSSubscriber
{
public:
  GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  ~GNSSSubscriber();
  void ParseData(std::deque<GNSSData> &deque_gnss_data);

private:
  void msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::deque<GNSSData> new_gnss_data_;
};
} 