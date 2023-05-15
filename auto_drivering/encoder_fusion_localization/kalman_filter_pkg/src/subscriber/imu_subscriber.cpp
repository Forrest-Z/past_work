/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:06:09
 * @LastEditors: luo
 * @LastEditTime: 2021-11-27 21:42:29
 */

#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"

namespace KalmanFilter
{
IMUSubscriber::IMUSubscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size)
    :nh_(nh) {
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMUSubscriber::msg_callback, this);
}

IMUSubscriber::~IMUSubscriber()
{
    
}

void IMUSubscriber::msg_callback(const sensor_msgs::ImuConstPtr& imu_msg_ptr) {
    IMUData imu_data;
    imu_data.time = imu_msg_ptr->header.stamp.toSec();

    imu_data.linear_acc.x = imu_msg_ptr->linear_acceleration.x;
    imu_data.linear_acc.y = imu_msg_ptr->linear_acceleration.y;
    imu_data.linear_acc.z = imu_msg_ptr->linear_acceleration.z;

    imu_data.angular_vel.x = imu_msg_ptr->angular_velocity.x;
    imu_data.angular_vel.y = imu_msg_ptr->angular_velocity.y;
    imu_data.angular_vel.z = imu_msg_ptr->angular_velocity.z;

    imu_data.orientation.x = imu_msg_ptr->orientation.x;
    imu_data.orientation.y = imu_msg_ptr->orientation.y;
    imu_data.orientation.z = imu_msg_ptr->orientation.z;
    imu_data.orientation.w = imu_msg_ptr->orientation.w;

    new_imu_data_.push_back(imu_data);

    // std::cout << "************msg_callback************new_imu_data_size = " << new_imu_data_.size() << std::endl;
}

void IMUSubscriber::ParseData(std::deque<IMUData>& imu_data_buff) {

    // std::cout << "***********************new_imu_data_size = " << new_imu_data_.size() << std::endl;
    if (new_imu_data_.size() > 0) 
    {
        
        imu_data_buff.insert(imu_data_buff.end(), new_imu_data_.begin(), new_imu_data_.end());
        
        new_imu_data_.clear();
    }
}
}