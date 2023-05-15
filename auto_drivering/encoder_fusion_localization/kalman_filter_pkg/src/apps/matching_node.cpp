/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-10 17:40:02
 * @LastEditors: luo
 * @LastEditTime: 2021-11-19 18:00:23
 */

#include <iostream>
#include <ros/ros.h>

#include "kalman_filter_pkg/matching/matching_flow.hpp"

int main(int argc, char* argv[])
{

    ros::init(argc, argv, "matching_node");

    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::MatchingFlow> matching_ptr = std::make_shared<KalmanFilter::MatchingFlow>(nh);

    ros::Rate rate(100);

    while(ros::ok())
    {
        ros::spinOnce();

        matching_ptr->Run();

        rate.sleep();

    }

    return 0;
}