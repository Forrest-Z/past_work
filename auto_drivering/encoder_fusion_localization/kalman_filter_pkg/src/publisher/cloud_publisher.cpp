/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:32:59
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:44:34
 */
#include "kalman_filter_pkg/publisher/cloud_publisher.hpp"


namespace KalmanFilter
{

CloudPublisher::CloudPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, size_t buff_size)
:nh_(nh), frame_id_(frame_id) {
publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

CloudPublisher::~CloudPublisher()
{

}

void CloudPublisher::Publish(CloudData::CloudPointTPtr& cloud_ptr_input, double time)
{
    ros::Time ros_time(time);
    PublishData(cloud_ptr_input, ros_time);

}
void CloudPublisher::Publish(CloudData::CloudPointTPtr& cloud_ptr_input)
{
    ros::Time time = ros::Time::now();
    PublishData(cloud_ptr_input, time);

}

bool CloudPublisher::HasSubscribers()
{
    return publisher_.getNumSubscribers() != 0;
}

void CloudPublisher::PublishData(CloudData::CloudPointTPtr& cloud_ptr_input, ros::Time time)
{
    sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
    pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);

    cloud_ptr_output->header.stamp = time;
    cloud_ptr_output->header.frame_id = frame_id_;
    publisher_.publish(*cloud_ptr_output);
    
}

}