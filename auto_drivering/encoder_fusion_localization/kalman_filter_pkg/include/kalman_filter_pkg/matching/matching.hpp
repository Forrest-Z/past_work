/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-09 16:53:47
 * @LastEditors: luo
 * @LastEditTime: 2021-12-29 09:58:37
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include <pcl/filters/random_sample.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/sensor_data/gnss_data.hpp"
#include "kalman_filter_pkg/subscriber/cloud_subscriber.hpp"
#include "kalman_filter_pkg/publisher/cloud_publisher.hpp"

#include "kalman_filter_pkg/models/cloud_filter/cloud_filter_interface.hpp"
#include "kalman_filter_pkg/models/cloud_filter/voxel_filter.hpp"
#include "kalman_filter_pkg/models/cloud_filter/box_filter.hpp"

#include "kalman_filter_pkg/models/registration/registration_interface.hpp"
#include "kalman_filter_pkg/models/registration/registration_icp.hpp"
#include "kalman_filter_pkg/models/registration/registration_ndt.hpp"



namespace KalmanFilter
{
struct KeyFramePoints{
    pcl::PointCloud<pcl::PointXYZ> GetTansformedCloud()
    {
        pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
        pcl::PointCloud<pcl::PointXYZ> cloud;
        Eigen::Matrix4d trans;
        trans.setIdentity();
        trans.block<3,3>(0,0) = quaternion_.toRotationMatrix();
        trans.block<3,1>(0,3) = position_;
        cloud += corner_cloud_;
        cloud += surf_cloud_;
        pcl::transformPointCloud(cloud,transformed_cloud,trans);
        return transformed_cloud;
    }

    double timestamp_;
    pcl::PointCloud<pcl::PointXYZ> corner_cloud_, surf_cloud_;
    Eigen::Vector3d position_;
    Eigen::Quaterniond quaternion_;
};

class Matching
{
public:
    Matching();
    ~Matching();

    bool InitWithConfig();
    bool InitDataPath(const YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);
    bool InitBoxFilter(const YAML::Node& config_node);
    bool InitKeyFrames(const YAML::Node& config_node);

    bool InitGlobalMap();
    bool ResetLocalMap(float x, float y, float z);

    void GetGlobalMap(CloudData::CloudPointTPtr& global_map);
    CloudData::CloudPointTPtr& GetLocalMap();
    CloudData::CloudPointTPtr& GetCurrentScan();

    bool LoadMap(CloudData::CloudPointTPtr& g_map, std::string map_path);

    bool HasNewGlobalMap();
    bool HasNewLocalMap(); 
    bool HasInited();
    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool Update(CloudData& cur_cloud_data, Eigen::Matrix4f& cur_pose);
private:

    std::shared_ptr<BoxFilter> box_filter_ptr_;

    std::shared_ptr<CloudFilterInterface> cloud_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_;
    std::shared_ptr<RegistrationInterface> global_registration_ptr_;

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    bool has_inited_ = false;


    std::string map_path_ = "";

    std::vector<KeyFramePoints> global_KFs_;
    CloudData::CloudPointTPtr global_map_ptr_;
    CloudData::CloudPointTPtr local_map_ptr_;
    CloudData::CloudPointTPtr current_scan_ptr_;

    bool has_new_global_map_ = false;
    bool has_new_local_map_ = false;


};


 
}