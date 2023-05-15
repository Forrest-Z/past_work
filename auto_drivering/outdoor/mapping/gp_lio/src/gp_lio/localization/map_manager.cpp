//
// Created by xc on 2020/12/4.
//

#include <pcl/point_cloud.h>
#include "gp_lio/localization/map_manager.h"

namespace gp_lio{
    bool MapManager::LoadMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& g_map,std::string map_path) {
        std::string dir_path = map_path + "/pose_graph";
        std::string file_path = dir_path + "/pose_graph.txt";
        std::string ground_cloud = "_ground.pcd";
        std::string less_edge_cloud = "_less_edge.pcd";
        std::string less_surf_cloud = "_less_surf.pcd";
        std::string outlier_cloud = "_outlier.pcd";

        std::cout << "load previous pose graph" << std::endl;
        std::FILE *ifile_pose_graph;
        ifile_pose_graph = std::fopen(file_path.c_str(), "r");
        if (ifile_pose_graph == NULL) {
            std::cout << "! Open [ " << file_path << " ] failed ..." << std::endl;
            return false;
        }
        global_KFs_.clear();
        KeyFrame tmp_kf;
        int i;
        double px, py, pz, qx, qy, qz, qw, vx, vy, vz, ax, ay, az, ba_1, ba_2, ba_3, bg_1, bg_2, bg_3, time_stamp,gps_la,gps_lo,gps_al;
        while (std::fscanf(ifile_pose_graph,"%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                           &i, &px, &py, &pz, &qx, &qy, &qz, &qw, &vx, &vy, &vz, &ax, &ay, &az,
                           &ba_1, &ba_2, &ba_3,&bg_1, &bg_2, &bg_3, &time_stamp, &gps_la, &gps_lo, &gps_al) != EOF) {
            tmp_kf.state_.timestamp_ = time_stamp;
            tmp_kf.state_.position_.x() = px;
            tmp_kf.state_.position_.y() = py;
            tmp_kf.state_.position_.z() = pz;
            tmp_kf.state_.quaternion_.x() = qx;
            tmp_kf.state_.quaternion_.y() = qy;
            tmp_kf.state_.quaternion_.z() = qz;
            tmp_kf.state_.quaternion_.w() = qw;
            tmp_kf.gnss_pose_.x() = floor(gps_la * 1e6);
            tmp_kf.gnss_pose_.y() = floor(gps_lo * 1e6);
            tmp_kf.gnss_pose_.z() = floor(gps_al * 1e6);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_surf_cloud,tmp_kf.cloud_feature_.less_surf_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_edge_cloud,tmp_kf.cloud_feature_.less_edge_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+ground_cloud,tmp_kf.cloud_feature_.ground_feature_);
            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+outlier_cloud,tmp_kf.cloud_feature_.outlier_feature_);
            *g_map += tmp_kf.GetTransformedAllCloud(extrinsic_parameters_);
            global_KFs_.push_back(tmp_kf);
        }
        std::fclose(ifile_pose_graph);
        std::cout << "load finished ... "<<std::endl;

        int kf_size = global_KFs_.size();
        pcl::PointCloud<pcl::PointXYZ>::Ptr GnssPoses3D (new pcl::PointCloud<pcl::PointXYZ>);
        GnssPoses3D->points.resize(kf_size);
        for(int i=0;i<kf_size;++i){
            GnssPoses3D->points[i].x = global_KFs_.at(i).gnss_pose_[0];
            GnssPoses3D->points[i].y = global_KFs_.at(i).gnss_pose_[1];
            GnssPoses3D->points[i].z = 0.0;
        }
        kdtreeGnssPose_.setInputCloud(GnssPoses3D);

        pcl::PointCloud<pcl::PointXYZ>::Ptr OdomPoses3D (new pcl::PointCloud<pcl::PointXYZ>);
        OdomPoses3D->points.resize(kf_size);
        for(int j=0;j<kf_size;++j){
            OdomPoses3D->points[j].x = global_KFs_.at(j).state_.position_[0];
            OdomPoses3D->points[j].y = global_KFs_.at(j).state_.position_[1];
            OdomPoses3D->points[j].z = global_KFs_.at(j).state_.position_[2];
        }
        kdtreeOdomPose_.setInputCloud(OdomPoses3D);
        std::cout << "ready ... "<<std::endl;
        return true;
    }

    bool MapManager::GetLocalMap(pcl::PointCloud<pcl::PointXYZI>::Ptr &local_cloud,
                                 double& traj_length,const Eigen::Vector3d& odom_p, int map_size) {
        int k = 1;
        std::vector<int> search_ind;
        std::vector<float> search_dis;
        pcl::PointXYZ odom_pose;
        odom_pose.x = odom_p[0];
        odom_pose.y = odom_p[1];
        odom_pose.z = odom_p[2];
        kdtreeOdomPose_.nearestKSearch(odom_pose,k,search_ind,search_dis);
        int index = 0;
        if(search_ind.size())
            index = search_ind[0];
        if(index == -1)
            return false;
        int kf_size = global_KFs_.size();
        int map_low_bound = (index-map_size) < 0 ? 0 : index-map_size;
        int map_up_bound = (index+map_size) > (kf_size-1) ? kf_size-1 : index+map_size;
        Eigen::Vector3d tmp_traj;
        for(int j = map_low_bound;j< map_up_bound;j++){
            *local_cloud += global_KFs_.at(j).GetTransformedAllCloud(extrinsic_parameters_);
            tmp_traj = global_KFs_.at(j+1).state_.position_ - global_KFs_.at(j).state_.position_;
            traj_length_ += tmp_traj.norm();
        }
        traj_length = traj_length_;
        *local_cloud += global_KFs_.at(map_up_bound).GetTransformedAllCloud(extrinsic_parameters_);
        if(local_cloud->points.size())
            return true;
        return false;
    }
    void MapManager::GetInitPose(const Eigen::Vector3d &gnss_pose, Eigen::Vector3d &odom_p,
                                 Eigen::Quaterniond &odom_q) {
        pcl::PointXYZ gnss_p;
        gnss_p.x = gnss_pose[0];
        gnss_p.y = gnss_pose[1];
        gnss_p.z = gnss_pose[2];
        int k = 3;
        std::vector<int> search_ind;
        std::vector<float> search_dis;
        kdtreeGnssPose_.nearestKSearch(gnss_p,k,search_ind,search_dis);
        int index_to_pose = 0;
        if(search_ind.size()){
            index_to_pose = search_ind[0]; // find the index of odom pose
            std::cout << "searched kf index: " << index_to_pose << " <--- "
                    << search_ind[0] << " "<< search_ind[1] << " "<< search_ind[2] << " "<<std::endl;
        }
        odom_p = global_KFs_.at(index_to_pose).state_.position_;
        odom_q = global_KFs_.at(index_to_pose).state_.quaternion_;
    }

}// namespace

