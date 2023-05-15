/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-01-13 21:04:25
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 21:06:50
 */
#pragma once

#include <iostream>
#include <ros/ros.h>

#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"

namespace KalmanFilter
{
class EncoderOdomFlow
{
public:
    EncoderOdomFlow();

    ~EncoderOdomFlow();

    bool Run();

    bool PublishData();


};

}