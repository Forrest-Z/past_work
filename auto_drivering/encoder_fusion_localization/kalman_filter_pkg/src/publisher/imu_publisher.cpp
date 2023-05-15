/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:33:14
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:44:51
 */

#include "kalman_filter_pkg/publisher/imu_publisher.hpp"


namespace KalmanFilter {

IMUPublisher::IMUPublisher(
    ros::NodeHandle& nh,
    std::string topic_name,
    std::string frame_id,
    size_t buff_size
)
:nh_(nh), frame_id_(frame_id) 
{
    publisher_ = nh_.advertise<sensor_msgs::Imu>(topic_name, buff_size);

    imu_.header.frame_id = frame_id_;


}
IMUPublisher::~IMUPublisher()
{

}

void IMUPublisher::Publish(const IMUData &imu_data, double time)
{
    ros::Time ros_time(time);
    PublishData(imu_data, ros_time);

}
void IMUPublisher::Publish(const IMUData &imu_data)
{
    ros::Time time = ros::Time::now();
    PublishData(imu_data, time);
}

bool IMUPublisher::HasSubscribers(void)
{
    return publisher_.getNumSubscribers() != 0;
}


void IMUPublisher::PublishData(const IMUData &imu_data, ros::Time time)
{
    imu_.header.stamp = time;

    // set orientation:
    imu_.orientation.w = imu_data.orientation.w;
    imu_.orientation.x = imu_data.orientation.x;
    imu_.orientation.y = imu_data.orientation.y;
    imu_.orientation.z = imu_data.orientation.z;

    // set angular velocity:
    imu_.angular_velocity.x = imu_data.angular_vel.x;
    imu_.angular_velocity.y = imu_data.angular_vel.y;
    imu_.angular_velocity.z = imu_data.angular_vel.z;

    // set linear acceleration:
    imu_.linear_acceleration.x = imu_data.linear_acc.x;
    imu_.linear_acceleration.y = imu_data.linear_acc.y;
    imu_.linear_acceleration.z = imu_data.linear_acc.z;

    publisher_.publish(imu_);

}
}