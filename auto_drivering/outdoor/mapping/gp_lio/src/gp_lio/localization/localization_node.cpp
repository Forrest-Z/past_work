//
// Created by xc on 2020/12/6.
//

#include <ros/ros.h>
#include"gp_lio/localization/localization_flow.h"
#include <gp_lio/localization/parameters.h>

int main(int argc,char**argv){
    ros::init(argc,argv,"localization_noe");
    std::string config_path = "/home/luo/work/catkin_ws/src/gp_lio/config/localization.yaml";
    ros::NodeHandle nh;
    gp_lio::LocalizationParameters localization_params;
    localization_params.ReadParameters(nh);
    gp_lio::LocalizationNode localization_node(nh,config_path);
    ros::Rate rate(100);
    while (ros::ok()){
        ros::spinOnce();
        localization_node.Run();
        rate.sleep();
    }
    return 0;
}




