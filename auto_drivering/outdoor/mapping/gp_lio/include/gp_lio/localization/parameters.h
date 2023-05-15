//
// Created by xc on 2020/12/6.
//

#ifndef GP_LIO_PARAMETERS_H
#define GP_LIO_PARAMETERS_H
#include <ros/ros.h>

#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace gp_lio{
    extern std::string MAP_PATH;
    extern double RES;
    extern double STEP_SIZE;
    extern double TRANS_EPS;
    extern double MAX_ITER;

    extern int LOCAL_MAP_SIZE;
    extern double GLOBAL_MAP_FILTER;
    extern double LOCAL_MAP_FILTER;
    extern double FRAME_FILTER;

    class LocalizationParameters
    {

    public:
        LocalizationParameters();

        ~LocalizationParameters();

        void ReadParameters(ros::NodeHandle &nh);

    public:
        // Path
        std::string ConfigPath_;
        std::string ResultPath_;

    private:
        template <typename T>
        T readParam(ros::NodeHandle &n, std::string name);
    };
}


#endif //GP_LIO_PARAMETERS_H
