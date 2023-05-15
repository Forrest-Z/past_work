/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:19:28
 * @LastEditors: luo
 * @LastEditTime: 2021-11-19 17:59:57
 */
#include <iostream>
#include <ros/ros.h>

#include "kalman_filter_pkg/data_pretreat/data_pretreat_flow.hpp"


int main (int argc, char* argv[])
{

    ros::init(argc, argv, "data_pretreat_node");

    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::DataPrestreatFlow> data_prestreat_ptr = std::make_shared<KalmanFilter::DataPrestreatFlow>(nh);

    ros::Rate rate(100);

    while (ros::ok())
    {
        ros::spinOnce();

        data_prestreat_ptr->Run();

        rate.sleep();
    }

    return 0;
}