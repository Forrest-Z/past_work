/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-24 20:49:12
 * @LastEditors: luo
 * @LastEditTime: 2022-01-06 11:05:48
 */

#include "kalman_filter_pkg/subscriber/encoder_subsciber.hpp"

namespace KalmanFilter
{

EncoderSubscriber::EncoderSubscriber(ros::NodeHandle &nh, std::string topic_name, size_t buff_size)
:nh_(nh)
{
    std::cout << "EncoderSubscriber 00" << std::endl;

    subscriber_ = nh_.subscribe(topic_name, buff_size, &EncoderSubscriber::msg_callback, this);

    std::cout << "EncoderSubscriber 11" << std::endl;
}


EncoderSubscriber::~EncoderSubscriber()
{


}

void EncoderSubscriber::msg_callback(const eqyc_joy2can::eqyc_IMCU_msgPtr& encoder_data_ptr)
{
    mutex.unlock();
    std::cout << "-----------------msg_callback 11" << std::endl;
    EncoderData encoder_data;
    encoder_data.time = encoder_data_ptr->header.stamp.toSec();

    encoder_data.control_mode_status = encoder_data_ptr->IMCU504_Controlmodestatus;
    encoder_data.vehicle_speed = encoder_data_ptr->IMCU504_Fbvehiclespeed;
    encoder_data.frequency_l1 = encoder_data_ptr->IMCU505_FrequencyL1;
    encoder_data.frequency_l2 = encoder_data_ptr->IMCU505_FrequencyL2;
    encoder_data.frequency_r1 = encoder_data_ptr->IMCU505_FrequencyR1;
    encoder_data.frequency_r2 = encoder_data_ptr->IMCU505_FrequencyR2;

    new_encoder_data_.push_back(encoder_data);

    std::cout << "callback size = " << new_encoder_data_.size() << std::endl;

    mutex.lock();
}


void EncoderSubscriber::ParseData(std::deque<EncoderData>& deque_encoder_data)
{
    mutex.unlock();

    if(new_encoder_data_.size() > 0)
    {
        deque_encoder_data.insert(deque_encoder_data.end(), new_encoder_data_.begin(), new_encoder_data_.end());

        new_encoder_data_.clear();
    }

    std::cout << " ParseData deque_encoder_data size = " << deque_encoder_data.size() << std::endl;
    mutex.lock();

}



} // namespace lidar_localization
