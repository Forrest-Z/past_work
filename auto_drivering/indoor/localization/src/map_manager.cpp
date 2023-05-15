//
// Created by xc on 2020/12/4.
//

#include <pcl/point_cloud.h>
#include "map_manager.h"

namespace gp_lio{
    bool MapManager::LoadMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& g_map,std::string map_path) {
        std::string map_save_dir_ = map_path + "/map";
        std::string pose_graph_file = map_save_dir_ + "/pose_graph.txt";
        std::string surf_cloud = "_surf_cloud.pcd";
        std::string corner_cloud = "_corner_cloud.pcd";
        std::FILE* ifile_pg;
        ifile_pg = std::fopen(pose_graph_file.c_str(), "r");
        if (ifile_pg == NULL) {
            std::cout << "! Open [ " << pose_graph_file << " ] failed ..." << std::endl;
            return false;
        }


        global_KFs_.clear();
        KeyFrame tmp_kf;
        int i;
        double t, px, py, pz, qx, qy, qz, qw;
        while(std::fscanf(ifile_pg,"%d %lf %lf %lf %lf %lf %lf %lf %lf", &i, &t, &px, &py, &pz, &qx, &qy, &qz, &qw) !=EOF){
            tmp_kf.timestamp_ = t;
            tmp_kf.position_.x() = px;
            tmp_kf.position_.y() = py;
            tmp_kf.position_.z() = pz;
            tmp_kf.quaternion_.x() = qx;
            tmp_kf.quaternion_.y() = qy;
            tmp_kf.quaternion_.z() = qz;
            tmp_kf.quaternion_.w() = qw;
            pcl::io::loadPCDFile(map_save_dir_ + "/" + std::to_string(i) + surf_cloud, tmp_kf.surf_cloud_);
            pcl::io::loadPCDFile(map_save_dir_ + "/" + std::to_string(i) + corner_cloud, tmp_kf.corner_cloud_);
//            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_surf_cloud,tmp_kf.cloud_feature_.less_surf_feature_);
//            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+less_edge_cloud,tmp_kf.cloud_feature_.less_edge_feature_);
//            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+ground_cloud,tmp_kf.cloud_feature_.ground_feature_);
//            pcl::io::loadPCDFile(dir_path+"/"+std::to_string(i)+outlier_cloud,tmp_kf.cloud_feature_.outlier_feature_);
            *g_map += tmp_kf.GetTansformedCloud();
            global_KFs_.push_back(tmp_kf);
        }
        std::fclose(ifile_pg);
        std::cout << "load pointcloud finished ... "<<std::endl;

        int kf_size = global_KFs_.size();
//        pcl::PointCloud<pcl::PointXYZ>::Ptr GnssPoses3D (new pcl::PointCloud<pcl::PointXYZ>);
//        GnssPoses3D->points.resize(kf_size);
//        for(int i=0;i<kf_size;++i){
//            GnssPoses3D->points[i].x = global_KFs_.at(i).gnss_pose_[0];
//            GnssPoses3D->points[i].y = global_KFs_.at(i).gnss_pose_[1];
//            GnssPoses3D->points[i].z = 0.0;
//        }
//        kdtreeGnssPose_.setInputCloud(GnssPoses3D);

        pcl::PointCloud<pcl::PointXYZ>::Ptr OdomPoses3D (new pcl::PointCloud<pcl::PointXYZ>);
        OdomPoses3D->points.resize(kf_size);
        for(int j=0;j<kf_size;++j){
            OdomPoses3D->points[j].x = global_KFs_.at(j).position_[0];
            OdomPoses3D->points[j].y = global_KFs_.at(j).position_[1];
            OdomPoses3D->points[j].z = global_KFs_.at(j).position_[2];
        }
        kdtreeOdomPose_.setInputCloud(OdomPoses3D);
        std::cout << "ready ... "<<std::endl;

        /*
        * * load aruco
        */
        std::string aruco_pos_file = map_save_dir_ + "/opt_aruco_data.txt";
        std::FILE *ifile_ar;
        ifile_ar = std::fopen(aruco_pos_file.c_str(), "r");
        if (ifile_ar == NULL) {
            std::cout << "! Open [ " << aruco_pos_file << " ] failed ..." << std::endl;
            return false;
        }

        int ar_i;
        double  ar_x, ar_y, ar_z, ar_qx, ar_qy, ar_qz, ar_qw;
        while (std::fscanf(ifile_ar, "%d %lf %lf %lf %lf %lf %lf %lf", &ar_i, &ar_x, &ar_y, &ar_z, &ar_qx, &ar_qy, &ar_qz, &ar_qw) !=EOF) {

            ArucoData aruco;
            aruco.id = ar_i;
            aruco.translation.x() = ar_x;
            aruco.translation.y() = ar_y;
            aruco.translation.z() = ar_z;
            aruco.orientation.x() = ar_qx;
            aruco.orientation.y() = ar_qy;
            aruco.orientation.z() = ar_qz;
            aruco.orientation.w() = ar_qw;
            aruco_vec_.push_back(aruco);
        }
        std::fclose(ifile_ar);
        std::cout << "load aruco finished ... "<<std::endl;
        std::cout << "load map finished ... "<<std::endl;

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
        std::cout << "GetLocalMap ---> odom_p " <<odom_p << std::endl;
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
            *local_cloud += global_KFs_.at(j).GetTansformedCloud();
            tmp_traj = global_KFs_.at(j+1).position_ - global_KFs_.at(j).position_;
            traj_length_ += tmp_traj.norm();
        }
        traj_length = traj_length_;
        *local_cloud += global_KFs_.at(map_up_bound).GetTansformedCloud();
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
        odom_p = global_KFs_.at(index_to_pose).position_;
        odom_q = global_KFs_.at(index_to_pose).quaternion_;
    }

    bool MapManager::GetGlobalArucoPose(uint& id ,ArucoData& global_aruco_pose){

        std::cout << "GetGlobalArucoPose --->  aruco_vec_ size : "<< aruco_vec_.size() << std::endl;
        for (auto marker : aruco_vec_)
        {
            if (marker.id == id)
            {
                global_aruco_pose = marker;
                return true;
            }
        }
//        std::cout << "not this aruco id data in map" << std::endl;
        return false;
    }

}// namespace

