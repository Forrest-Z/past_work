/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-26 14:50:36
 * @LastEditors: luo
 * @LastEditTime: 2021-11-27 21:14:29
 */

#include <iostream>
#include <ros/ros.h>

#include "kalman_filter_pkg/imu_encoder_eskf/imu_encoder_eskf.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_encoder_eskf_node");
    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::ImuEncoderESKF> imu_encoder_ptr = std::make_shared<KalmanFilter::ImuEncoderESKF>(nh);


    ros::Rate rate(100);

    while( ros::ok())
    {
        ros::spinOnce();

        imu_encoder_ptr->Run();

        rate.sleep();

    }

    return 0;

}