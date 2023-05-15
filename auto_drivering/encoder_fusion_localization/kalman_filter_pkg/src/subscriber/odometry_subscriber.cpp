/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-08 09:39:06
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 16:12:34
 */
#include "kalman_filter_pkg/subscriber/odometry_subscriber.hpp"

namespace KalmanFilter{
    
OdometrySubscriber::OdometrySubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &OdometrySubscriber::Msg_CallBack, this);
}

OdometrySubscriber::~OdometrySubscriber()
{

}

void OdometrySubscriber::Msg_CallBack(const nav_msgs::OdometryConstPtr& odom_msg_ptr) {
    buff_mutex_.lock();
    PoseData pose_data;
    pose_data.time = odom_msg_ptr->header.stamp.toSec();

    // set the position:
    pose_data.pose(0,3) = odom_msg_ptr->pose.pose.position.x;
    pose_data.pose(1,3) = odom_msg_ptr->pose.pose.position.y;
    pose_data.pose(2,3) = odom_msg_ptr->pose.pose.position.z;

    // set the orientation:
    Eigen::Quaternionf q;
    q.x() = odom_msg_ptr->pose.pose.orientation.x;
    q.y() = odom_msg_ptr->pose.pose.orientation.y;
    q.z() = odom_msg_ptr->pose.pose.orientation.z;
    q.w() = odom_msg_ptr->pose.pose.orientation.w;
    pose_data.pose.block<3,3>(0,0) = q.matrix();

    // set the linear velocity:
    pose_data.vel.x() = odom_msg_ptr->twist.twist.linear.x;
    pose_data.vel.y() = odom_msg_ptr->twist.twist.linear.y;
    pose_data.vel.z() = odom_msg_ptr->twist.twist.linear.z;

    pose_data.Vel.v.x() = odom_msg_ptr->twist.twist.linear.x;
    pose_data.Vel.v.y() = odom_msg_ptr->twist.twist.linear.y;
    pose_data.Vel.v.z() = odom_msg_ptr->twist.twist.linear.z;

    pose_data.Vel.w.x() = odom_msg_ptr->twist.twist.angular.x;
    pose_data.Vel.w.y() = odom_msg_ptr->twist.twist.angular.y;
    pose_data.Vel.w.z() = odom_msg_ptr->twist.twist.angular.z;

    // new_odom_data_.push_back(*odom_msg_ptr);
    new_pose_data_.push_back(pose_data);
    
    buff_mutex_.unlock();
}

void OdometrySubscriber::ParseData(std::deque<PoseData>& pose_data_buff) {
    buff_mutex_.lock();
    if (new_pose_data_.size() > 0) {
        pose_data_buff.insert(pose_data_buff.end(), new_pose_data_.begin(), new_pose_data_.end());
        new_pose_data_.clear();
    }
    buff_mutex_.unlock();
}

// void OdometrySubscriber::ParseOdomData(std::deque<nav_msgs::Odometry>& deque_odom_data)
// {
//     buff_mutex_.lock();
//     if (new_odom_data_.size() > 0) {
//         deque_odom_data.insert(deque_odom_data.end(), new_odom_data_.begin(), new_odom_data_.end());
//         new_odom_data_.clear();
//     }
//     buff_mutex_.unlock();
    
// }

}