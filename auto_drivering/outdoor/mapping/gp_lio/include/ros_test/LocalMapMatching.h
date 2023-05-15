//
// Created by wchen on 2020/7/20.
//

#ifndef GP_LIO_LOCALMAPMATCHING_H
#define GP_LIO_LOCALMAPMATCHING_H

#include <iostream>
#include <string>

#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/utility/Utility.h"

#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

// ceres
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>

namespace gp_lio{

    struct Plane2D{

        Eigen::Vector2f line_left_;
        Eigen::Vector2f line_right_;
        Eigen::Vector2f cp_;

    };

    class LocalMapMatching {

    public:
        LocalMapMatching();

        ~LocalMapMatching();

        void LoadData();

        void Run();

    private:

        void GroundRegistration();

        void PlaneRegistration();

        void AllCloudRegistration();

        RegistrationResult ICPRegistration(pcl::PointCloud<pcl::PointXYZI> source_cloud, pcl::PointCloud<pcl::PointXYZI> target_cloud, Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity());

        void ShowClouds(PointCloudT ref_cloud, PointCloudT query_cloud, std::string type);

        PointCloudT ChangeCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

    private:
        pcl::PointCloud<pcl::PointXYZI> ref_cloud_;
        pcl::PointCloud<pcl::PointXYZI> query_cloud_;

        pcl::PointCloud<pcl::PointXYZI> ref_ground_cloud_;
        pcl::PointCloud<pcl::PointXYZI> query_ground_cloud_;

        pcl::PointCloud<pcl::PointXYZI> ref_plane_cloud_;
        pcl::PointCloud<pcl::PointXYZI> query_plane_cloud_;

        std::vector<Plane2D> ref_planes_;
        std::vector<Plane2D> query_planes_;

        pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_XYZI_;

        pcl::visualization::PCLVisualizer::Ptr cloud_viewer_;

        int loop_count_;
        int ref_idx_;
        int query_idx_;

        Eigen::Matrix4f align_mat_;

    };

}



#endif //GP_LIO_LOCALMAPMATCHING_H
