/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:32:33
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:39:10
 */
#pragma once

#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"

namespace KalmanFilter {
class IMUPublisher {
  public:
    IMUPublisher(
      ros::NodeHandle& nh,
      std::string topic_name,
      std::string frame_id,
      size_t buff_size
    );
    ~IMUPublisher();

    void Publish(const IMUData &imu_data, double time);
    void Publish(const IMUData &imu_data);

    bool HasSubscribers(void);

  private:
    void PublishData(const IMUData &imu_data, ros::Time time);

    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

    sensor_msgs::Imu imu_;
};
} 