/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-02 15:14:01
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:39:13
 */
#pragma once

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "kalman_filter_pkg/sensor_data/velocity_data.hpp"

namespace KalmanFilter
{
class OdometryPublisher
{
public:
    OdometryPublisher(ros::NodeHandle& nh, std::string topic_name, std::string base_frame_id, std::string child_frame_id, size_t buff_size);

    ~OdometryPublisher();

    // void Publish(const Eigen::Matrix4f& transform_matrix);

    void Publish(const Eigen::Matrix4f& transform_matrix, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const VelocityData &velocity_data);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel, double time);
    void Publish(const Eigen::Matrix4f& transform_matrix, const Eigen::Vector3f& vel);

    bool HasSubscribers();

private:
    void PublishData(
      const Eigen::Matrix4f& transform_matrix, 
      const VelocityData &velocity_data, 
      ros::Time time
    );
private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;

    VelocityData velocity_data_;
    nav_msgs::Odometry odometry_;

};

}