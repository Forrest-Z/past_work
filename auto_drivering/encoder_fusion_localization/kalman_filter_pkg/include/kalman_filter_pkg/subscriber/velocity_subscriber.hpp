/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:51:16
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:41:34
 */
#pragma once

#include <deque>
#include <mutex>
#include <thread>

#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"

#include "kalman_filter_pkg/sensor_data/velocity_data.hpp"

namespace KalmanFilter {
class VelocitySubscriber {
  public:
    VelocitySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
    VelocitySubscriber() = default;
    void ParseData(std::deque<VelocityData>& deque_velocity_data);

  private:
    void msg_callback(const geometry_msgs::TwistStampedConstPtr& twist_msg_ptr);

  private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    std::deque<VelocityData> new_velocity_data_;

    std::mutex buff_mutex_; 
};
}