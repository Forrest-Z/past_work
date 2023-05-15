/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 18:02:24
 * @LastEditors: luo
 * @LastEditTime: 2021-12-22 10:11:40
 */

#include <Eigen/Dense>

#include "kalman_filter_pkg/publisher/key_frames_publisher.hpp"

namespace KalmanFilter
{

KeyFramesPublisher::KeyFramesPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size)
:nh_(nh), frame_id_(frame_id)
{
    pub_ = nh_.advertise<nav_msgs::Path>(topic_name, buff_size);

}

KeyFramesPublisher::~KeyFramesPublisher()
{

}

void KeyFramesPublisher::Publish(const std::deque<KeyFrame>& key_frames)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;

    for(size_t i = 0; i < key_frames.size(); ++i)
    {
        KeyFrame key_frame = key_frames.at(i);

        geometry_msgs::PoseStamped pose_stamped;
        ros::Time ros_time(key_frame.time);
        pose_stamped.header.stamp = ros_time;
        pose_stamped.header.frame_id = frame_id_;

        pose_stamped.header.seq = key_frame.index;

        pose_stamped.pose.position.x = key_frame.pose(0, 3);
        pose_stamped.pose.position.y = key_frame.pose(1, 3);
        pose_stamped.pose.position.z = key_frame.pose(2, 3);

        Eigen::Quaternionf q = key_frame.GetQuaternion();
        pose_stamped.pose.orientation.w = q.w();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();

        path.poses.push_back(pose_stamped);
    }

    pub_.publish(path);


}

bool KeyFramesPublisher::HasSubscriber()
{

    return pub_.getNumSubscribers() != 0;

}


}