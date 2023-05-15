/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-18 17:17:15
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 19:59:17
 */
#include <iostream>
#include <ros/ros.h>

#include "kalman_filter_pkg/kalman_filter/kalman_filter_flow.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kalman_filter_node");

    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::KalmanFilterFlow> kalman_filter_ptr = std::make_shared<KalmanFilter::KalmanFilterFlow>(nh);
    
    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();

        kalman_filter_ptr->Run();

        rate.sleep();

    }
    std::cout << "kalman_filter_node" << std::endl;

    return 0;
}