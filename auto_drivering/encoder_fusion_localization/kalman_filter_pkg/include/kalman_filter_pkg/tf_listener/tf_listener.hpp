/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-02 09:57:51
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:42:04
 */
#pragma once

#include <string>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <tf/transform_listener.h>

namespace KalmanFilter
{
class TFListener
{
public:
    TFListener(ros::NodeHandle& nh, std::string base_frame_id, std::string child_frame_id);

    ~TFListener();

    bool LookupData(Eigen::Matrix4f& transform_matrix);


private:
    bool TransformToMatrix(const tf::StampedTransform& transform, Eigen::Matrix4f& transform_matrix);

private:
    ros::NodeHandle nh_;
    tf::TransformListener listener_;
    std::string base_frame_id_;
    std::string child_frame_id_;

};
}