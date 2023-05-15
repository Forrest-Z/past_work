/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:40
 * @LastEditors: luo
 * @LastEditTime: 2022-01-05 17:41:56
 */
#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"

namespace KalmanFilter
{
CloudSubscriber::CloudSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
    : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &CloudSubscriber::msg_callback, this);
}

CloudSubscriber::~CloudSubscriber()
{
    
}

void CloudSubscriber::msg_callback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg_ptr)
{
    CloudData cloud_data;
    cloud_data.time = cloud_msg_ptr->header.stamp.toSec();
    pcl::fromROSMsg(*cloud_msg_ptr, *(cloud_data.cloud_ptr));

    // CloudData cloud_points;
    // cloud_points.time = cloud_msg_ptr->header.stamp.toSec();
    // cloud_points.
    


    new_cloud_data_.push_back(cloud_data);
    // std::cout << "------------msg_callback size = " << new_cloud_data_.size() << std::endl;
}

void CloudSubscriber::ParseData(std::deque<CloudData> &cloud_data_buff)
{
    // std::cout << "------------new_cloud_data_ size = " << new_cloud_data_.size() << std::endl;

    if (new_cloud_data_.size() > 0)
    {
        cloud_data_buff.insert(cloud_data_buff.end(), new_cloud_data_.begin(), new_cloud_data_.end());
        new_cloud_data_.clear();
    }
}
}