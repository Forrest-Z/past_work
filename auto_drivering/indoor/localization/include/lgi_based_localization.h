//
// Created by xc on 2020/12/4.
//

#ifndef GP_LIO_LGI_BASED_LOCALIZATION_H
#define GP_LIO_LGI_BASED_LOCALIZATION_H

#include <string>
// pcl
#include<pcl/registration/ndt.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
// Eigen
#include <Eigen/Dense>
//
#include <yaml-cpp/yaml.h>

// local
#include "map_manager.h"
#include "parameters.h"



namespace gp_lio{
    class Localization{
    public:
        Localization();
        ~Localization();
        bool Update(pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan,Eigen::Matrix4f& cloud_pose);
        void GaussPoseByGnss(Eigen::Vector3d& gnss_pose);
        bool GaussPoseByAruco(ArucoData aruco_pose,Eigen::Matrix4f& current_gauss_pose);
        bool GetGlobalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& global_map);
        bool GetLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& local_map);
        bool GetCurrentScan(pcl::PointCloud<pcl::PointXYZI>::Ptr& current_scan);
        bool HasInited();
        bool HasNewGlobalMap();
        bool HasNewLocalMap();

    private:
        bool InitWithConfig();
        bool SetInitPose(const Eigen::Matrix4f& init_pose);
        bool InitGlobalMap();
        bool ResetLocalMap(const Eigen::Vector3d& frame_pose);
        void SetNdtParam(float res,float step_size,float trans_eps, int max_iter);
        bool ScanMatch(const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,const Eigen::Matrix4f& predict_pose,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& result_cloud,Eigen::Matrix4f& result_pose);

    private:
        std::string map_path_ = MAP_PATH;

        pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>::Ptr ndt_ptr_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr local_map_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_scan_;

        pcl::VoxelGrid<pcl::PointXYZI> global_map_filter_;
        pcl::VoxelGrid<pcl::PointXYZI> local_map_filter_;
        pcl::VoxelGrid<pcl::PointXYZI> current_frame_filter_;

        Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f current_gauss_pose_ = Eigen::Matrix4f::Identity();

        int local_map_size_;      // It's keyFrames size actually
        double all_local_map_length_ = 0.0; // trajectory length of keyFrame about local map
        double forward_length_ = 0.0;

        bool has_inited_;
        bool has_new_gloabl_map_;
        bool has_new_local_map_;
        MapManager map_manager_;
        Eigen::Matrix4f cam2lidar_;

    };

}


#endif //GP_LIO_LGI_BASED_LOCALIZATION_H
