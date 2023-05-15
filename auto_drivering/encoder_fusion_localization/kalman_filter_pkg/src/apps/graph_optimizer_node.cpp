/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:42:08
 * @LastEditors: luo
 * @LastEditTime: 2022-01-19 14:52:57
 */
#include <iostream>
#include <ros/ros.h>
#include <deque>

#include "kalman_filter_pkg/graph_optimizer/graph_optimizer_flow.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "graph_optimizer_node");

    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::GraphOptimizerFlow> graph_optimizer_ptr = 
                                    std::make_shared<KalmanFilter::GraphOptimizerFlow>(nh);

    ros::Rate rate(200);
    
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();

    while(ros::ok())
    {
        ros::spinOnce();

        graph_optimizer_ptr->Run();
        
        rate.sleep();
    }


    return 0;
}