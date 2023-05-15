/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:32:46
 * @LastEditors: luo
 * @LastEditTime: 2021-11-19 18:12:30
 */

#pragma once

#include <string>

#include <Eigen/Dense>

#include <ros/ros.h>

#include "/home/luo/work/filter_ws/devel/include/kalman_filter_pkg/PosVel.h"
// #include "fusion_localization/PosVel.h"

#include "kalman_filter_pkg/sensor_data/pose_vel_data.hpp"

// using namespace fusion_localization;

namespace KalmanFilter {

class PosVelPublisher {
  public:
    PosVelPublisher(
      ros::NodeHandle& nh, 
      std::string topic_name, 
      std::string base_frame_id,
      std::string child_frame_id,
      int buff_size
    );
    ~PosVelPublisher();

    void Publish(const PosVelData &pos_vel_data, const double &time);
    void Publish(const PosVelData &pos_vel_data);

    bool HasSubscribers();

  private:
    void PublishData( const PosVelData &pos_vel_data, ros::Time time );

  private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    // PosVel pos_vel_msg_;

    kalman_filter_pkg::PosVel pos_vel_msg_;
};

} // namespace lidar_localization
