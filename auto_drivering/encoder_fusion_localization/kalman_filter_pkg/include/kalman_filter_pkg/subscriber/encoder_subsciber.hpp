/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-24 20:46:32
 * @LastEditors: luo
 * @LastEditTime: 2021-11-27 21:57:25
 */
#pragma once


#include <deque>
#include <iostream>
#include <mutex>
#include <thread>

#include <ros/ros.h>

// #include "/home/luo/work/filter_ws/devel/include/eqyc_joy2can/eqyc_IMCU_msg.h"
#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"


namespace KalmanFilter
{
class EncoderSubscriber
{
public:
  EncoderSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size);
  ~EncoderSubscriber();
  void ParseData(std::deque<EncoderData> &deque_encoder_data);

private:
  void msg_callback(const eqyc_joy2can::eqyc_IMCU_msgPtr& encoder_data_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;

  std::mutex mutex;

  std::deque<EncoderData> new_encoder_data_;
};
} // namespace lidar_localization
