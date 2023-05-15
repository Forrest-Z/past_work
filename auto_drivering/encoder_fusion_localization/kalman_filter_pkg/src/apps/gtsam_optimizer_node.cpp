/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-30 14:36:01
 * @LastEditors: luo
 * @LastEditTime: 2022-01-19 11:37:43
 */
#include <iostream>
#include <ros/ros.h>
#include <deque>
// #include <boost/shared_ptr.hpp>

#include "kalman_filter_pkg/gtsam_optimizer/gtsam_optimizer_flow.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gtsan_optimizer_node");

    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::GtsamOptimizer> gtsam_optimizer_ptr = std::make_shared<KalmanFilter::GtsamOptimizer>(nh);

    ros::Rate rate(200);

    while(ros::ok())
    {
        ros::spinOnce();

        gtsam_optimizer_ptr->Run();

        rate.sleep();
    }

    ros::MultiThreadedSpinner spinner(4);

    return 0;
}