/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-02-10 18:04:43
 * @LastEditors: luo
 * @LastEditTime: 2022-02-10 19:38:41
 */
//
// Created by xc on 2020/12/6.
//

#include <ros/ros.h>
#include"localization_flow.h"
#include "parameters.h"
#include <time.h>

int main(int argc,char**argv){
    ros::init(argc,argv,"localization_noe");
    std::string config_path = "/home/luo/work/map/work_ws/src/localization/config/localization.yaml";
    ros::NodeHandle nh;
    clock_t t0, t1, t2, t3, t4, t5;
    // t0 = clock();
    gp_lio::LocalizationParameters localization_params;
    // t1 = clock();
    localization_params.ReadParameters(nh);
    // t2 = clock();

    gp_lio::LocalizationNode localization_node(nh);
    // t3 = clock();

    // std::cout<< " time1 = " << (double)(t1 - t0) / CLOCKS_PER_SEC << std::endl;
    // std::cout<< " time2 = " << (double)(t2 - t1) / CLOCKS_PER_SEC << std::endl;
    // std::cout<< " time3 = " << (double)(t3 - t2) / CLOCKS_PER_SEC << std::endl;
    ros::Rate rate(100);
    while (ros::ok()){
        ros::spinOnce();
//        sleep(5);
// t4 = clock();
        localization_node.Run();
// t5 = clock();

// std::cout<< "node time = " << (double)(t5 - t4) / CLOCKS_PER_SEC << std::endl;
        rate.sleep();
    }
    return 0;
}




