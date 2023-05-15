/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:59
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:46:11
 */

#include "kalman_filter_pkg/subscriber/gnss_subscriber.hpp"

namespace KalmanFilter
{
GNSSSubscriber::GNSSSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
    : nh_(nh)
{
    subscriber_ = nh_.subscribe(topic_name, buff_size, &GNSSSubscriber::msg_callback, this);
}

GNSSSubscriber::~GNSSSubscriber()
{
    
}


void GNSSSubscriber::msg_callback(const sensor_msgs::NavSatFixConstPtr &nav_sat_fix_ptr)
{
    GNSSData gnss_data;
    gnss_data.time = nav_sat_fix_ptr->header.stamp.toSec();
    gnss_data.latitude = nav_sat_fix_ptr->latitude;
    gnss_data.longitude = nav_sat_fix_ptr->longitude;
    gnss_data.altitude = nav_sat_fix_ptr->altitude;
    gnss_data.status = nav_sat_fix_ptr->status.status;
    gnss_data.service = nav_sat_fix_ptr->status.service;

    new_gnss_data_.push_back(gnss_data);
}

void GNSSSubscriber::ParseData(std::deque<GNSSData> &gnss_data_buff)
{
    if (new_gnss_data_.size() > 0)
    {
        gnss_data_buff.insert(gnss_data_buff.end(), new_gnss_data_.begin(), new_gnss_data_.end());
        new_gnss_data_.clear();
    }
}

}