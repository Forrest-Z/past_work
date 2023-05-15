/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-31 10:55:20
 * @LastEditors: luo
 * @LastEditTime: 2022-01-19 12:50:07
 */
#include <iostream>
#include <thread>

#include <ros/ros.h>

#include "kalman_filter_pkg/gtsam_optimizer/gtsam_odom_optimizer_flow.hpp"

int main(int argc, char** argv){

    // google::InitGoogleLogging(argv[0]);
    // FLAGS_alsologtostderr = 1;

    ros::init(argc,argv,"gtsam_odom_optimizer_node");
    
    ros::NodeHandle nh;

    std::shared_ptr<KalmanFilter::IMUOdompreintegration> gtsam_odom_optimizer_ptr = 
                                    std::make_shared<KalmanFilter::IMUOdompreintegration>(nh);

    // ros::MultiThreadedSpinner spinner(4);

    ros::Rate rate(200);
    while (ros::ok()){
        // spinner.spin();
        ros::spinOnce();

        gtsam_odom_optimizer_ptr->run();

        rate.sleep();
    }



    return 0;

}