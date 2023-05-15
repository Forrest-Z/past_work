/*
 * @Author: your name
 * @Date: 2021-09-07 16:29:35
 * @LastEditTime: 2021-09-23 14:23:22
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /ws_catkin/src/odom_pkg/src/odom_node.cpp
 */

#include <ros/ros.h>
#include <iostream>

#include "odom_pkg/odom_flow.hpp"

using namespace std;

// class WheelOdomMsg;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "odom_node");

    ros::NodeHandle nh;

    // WheelOdomMsg wheel_odom(nh);
    std::shared_ptr<WheelOdomMsg> wheel_odom_ptr = std::make_shared<WheelOdomMsg>(nh);

    ros::Rate rate(100);
    while (ros::ok())
    {
        ros::spinOnce();

        // wheel_odom.Process();
        wheel_odom_ptr->Process();

        rate.sleep();
    }
    
    return 0;
}