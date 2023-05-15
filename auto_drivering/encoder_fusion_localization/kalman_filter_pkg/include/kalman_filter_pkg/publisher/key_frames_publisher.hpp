/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 18:02:43
 * @LastEditors: luo
 * @LastEditTime: 2021-12-21 18:06:43
 */
#pragma once

#include <string>
#include <deque>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include "kalman_filter_pkg/sensor_data/key_frame.hpp"

namespace KalmanFilter
{
class KeyFramesPublisher
{
public:
    KeyFramesPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size);
    ~KeyFramesPublisher();

    void Publish(const std::deque<KeyFrame>& key_frames);

    bool HasSubscriber();

private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    std::string frame_id_ = "";
};
}