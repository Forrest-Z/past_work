/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:32:20
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:39:23
 */
#pragma once

#include <string>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"

namespace KalmanFilter
{
class CloudPublisher
{
public:
    CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size);

    ~CloudPublisher();

    void Publish(CloudData::CloudPointTPtr& cloud_ptr_input, double time);
    void Publish(CloudData::CloudPointTPtr& cloud_ptr_input);

    bool HasSubscribers();

private:
    void PublishData(CloudData::CloudPointTPtr& cloud_ptr_input, ros::Time time);

private:
    ros::NodeHandle nh_;
    ros::Publisher publisher_;
    std::string frame_id_;

};

}