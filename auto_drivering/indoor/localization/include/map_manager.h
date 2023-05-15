//
// Created by xc on 2020/12/4.
//

#ifndef GP_LIO_MAP_MANAGER_H
#define GP_LIO_MAP_MANAGER_H

#include <iostream>
#include <vector>
#include <array>
// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
//local
#include "KeyFrame.h"


namespace gp_lio{

    struct KFrame{
        Eigen::Vector3d odom_position;
        Eigen::Quaterniond odom_quaternion;
        Eigen::Vector3d gnss_pose;
        pcl::PointCloud<pcl::PointXYZI> cloud;
    };

    struct ArucoData{
        ArucoData(){
            id = 0;
            time = 0.0;
            translation.setZero();
            orientation.setIdentity();
        }
        Eigen::Matrix4f getPose(){
            Eigen::Matrix4f pose;
            pose.setIdentity();
            pose.block<3,3>(0,0) = orientation.normalized().toRotationMatrix();
            pose.block<3,1>(0,3) = translation;
            return pose;
        }
        uint id;
        double time;
        Eigen::Vector3f translation;
        Eigen::Quaternionf orientation;
    };

    class MapManager{
    public:
        bool GetLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& local_cloud,double& traj_length,const Eigen::Vector3d& odom_p,int map_size);
        bool LoadMap(pcl::PointCloud<pcl::PointXYZI>::Ptr&,std::string map_path);
        void GetInitPose(const Eigen::Vector3d& gnss_pose,Eigen::Vector3d& odom_p,Eigen::Quaterniond& odom_q);
        bool GetGlobalArucoPose(uint& id ,ArucoData& global_aruco_pose);

    private:
        double traj_length_=0.0;
        std::vector<KeyFrame> global_KFs_;
        std::vector<ArucoData> aruco_vec_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeGnssPose_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeOdomPose_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
        ExternalParameters extrinsic_parameters_;


    };
}

#endif //GP_LIO_MAP_MANAGER_H
