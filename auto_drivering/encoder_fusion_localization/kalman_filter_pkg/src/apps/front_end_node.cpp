/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:08:27
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 18:15:58
 */

#include <memory>
#include <iostream>
#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <glog/logging.h>

#include "kalman_filter_pkg/front_end/front_end_flow.hpp"
#include "kalman_filter_pkg/global_defination/global_defination.h"

int main(int argc, char* argv[])
{
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_log_dir = WORK_SPACE_PATH + "/Log";
    // FLAGS_alsologtostderr = 1;

    ros::init(argc, argv, "front_end_node");

    ros::NodeHandle nh;
    
    std::shared_ptr<KalmanFilter::FrontEndFlow> front_end_ptr = std::make_shared<KalmanFilter::FrontEndFlow>(nh);

    ros::Rate rate(100);

    while(ros::ok())
    {

        ros::spinOnce();

        front_end_ptr->Run();

        rate.sleep();
    }

    std::cout<< "front_end_node" << std::endl;

    return 0;
}