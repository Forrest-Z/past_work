/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-02-10 18:04:43
 * @LastEditors: luo
 * @LastEditTime: 2022-02-10 19:38:31
 */
//
// Created by xc on 2020/12/6.
//

#include "parameters.h"

namespace gp_lio{
    std::string MAP_PATH;
    double RES;
    double STEP_SIZE;
    double TRANS_EPS;
    double MAX_ITER;

    int LOCAL_MAP_SIZE;
    double GLOBAL_MAP_FILTER;
    double LOCAL_MAP_FILTER;
    double FRAME_FILTER;


    LocalizationParameters::LocalizationParameters()
    {
    }

    LocalizationParameters::~LocalizationParameters()
    {
    }

    template <typename T>
    T LocalizationParameters::readParam(ros::NodeHandle &n, std::string name)
    {
        T ans;
        if (n.getParam(name, ans))
        {
            ROS_INFO_STREAM("Loaded " << name << ": " << ans);
        }
        else
        {
            ROS_ERROR_STREAM("Failed to load " << name);
            n.shutdown();
        }
        return ans;
    }

    void LocalizationParameters::ReadParameters(ros::NodeHandle &nh)
    {
        std::string config_file = "/home/luo/work/map/work_ws/src/localization/config/localization.yaml";
//        std::string config_file = readParam<std::string>(nh, "config_file");
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            std::cerr << "ERROR: Wrong path to settings" << std::endl;
        }
        fsSettings["map_path"] >> MAP_PATH;
        RES = fsSettings["res"];
        STEP_SIZE = fsSettings["step_size"];
        TRANS_EPS = fsSettings["trans_eps"];
        MAX_ITER = fsSettings["max_iter"];

        LOCAL_MAP_SIZE = fsSettings["local_map_size"];
        GLOBAL_MAP_FILTER = fsSettings["global_map_filter"];
        LOCAL_MAP_FILTER = fsSettings["local_map_filter"];
        FRAME_FILTER = fsSettings["frame_filter"];
        fsSettings.release();
    }
}