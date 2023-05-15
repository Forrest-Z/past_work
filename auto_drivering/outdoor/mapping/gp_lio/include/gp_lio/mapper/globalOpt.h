//
// Created by xc on 2021/2/22.
//

#ifndef GP_LIO_GLOBALOPT_H
#define GP_LIO_GLOBALOPT_H

#endif //GP_LIO_GLOBALOPT_H
// std
#include <array>
#include <map>
#include <thread>
#include <mutex>
// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
// ceres
#include <ceres/ceres.h>
// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
// ROS
#include "sensor_msgs/PointCloud2.h"
#include "LocalCartesian.hpp"
#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/factor/GpPriorFactor.h"

namespace gp_lio{
    class GlobalOptimization{
    public:
        GlobalOptimization();
        ~GlobalOptimization();
        void inputOdom(double t,Eigen::Vector3d odomP,Eigen::Quaterniond odomQ);
        void inputGPS(double t,double latitude, double longitude, double altitude,double pos_accuracy);
        void getGlobalOdom(Eigen::Vector3d &odmmP,Eigen::Quaterniond &odomQ);
        void getGlobalPath(pcl::PointCloud<pcl::PointXYZ>& global_path);



    private:
        void gps2XYZ(double latitude, double longitude, double altitude, double* xyz);
        void optimize();
        void updateGlobalPath();

    private:
        GeographicLib::LocalCartesian geoConverter_;
        bool initGPS_, newGPS_;
        std::map<double,std::array<double ,7>> local_pose_v;
        std::map<double,std::array<double ,7 >> gloabl_pose_v;
        std::map<double,std::array<double,4>> gps_pose_v;
        Eigen::Matrix4d WGPS_T_WVIO;
        std::thread threadOpt_;
        std::mutex mpose_;
        pcl::PointCloud<pcl::PointXYZ> global_path_;
    };

}



