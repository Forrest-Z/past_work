/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:09:26
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 18:16:34
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>

#include <yaml-cpp/yaml.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
#include "kalman_filter_pkg/models/cloud_filter/cloud_filter_interface.hpp"
#include "kalman_filter_pkg/models/registration/registration_interface.hpp"
#include "kalman_filter_pkg/models/registration/registration_icp.hpp"
#include "kalman_filter_pkg/models/registration/registration_ndt.hpp"

#include "kalman_filter_pkg/models/cloud_filter/voxel_filter.hpp"

// #include "fusion_localization/subscriber/imu_subscriber.hpp"
// #include "fusion_localization/subscriber/gnss_subscriber.hpp"
// #include "fusion_localization/subscriber/cloud_subscriber.hpp"
// #include "fusion_localization/tf_listener/tf_listener.hpp"


namespace KalmanFilter
{
class FrontEnd
{

public:
    struct Frame{
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
        CloudData cloud_data;
    };

public:
    FrontEnd();
    ~FrontEnd();

    bool InitWithConfig();
    bool InitDataPath(YAML::Node& config_node);
    bool InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node);
    bool InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node);

    bool SetInitPose(const Eigen::Matrix4f& init_pose);
    bool Update(CloudData& current_cloud_data, Eigen::Matrix4f& cloud_pose);

    bool UpdateWithNewFrame(const Frame& new_key_frame);

private:
    ros::NodeHandle nh_;

    std::string data_path_ = "";

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();

    std::shared_ptr<CloudFilterInterface> frame_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> local_map_filter_ptr_;
    std::shared_ptr<CloudFilterInterface> global_map_filter_ptr_;

    std::shared_ptr<RegistrationInterface> registration_ptr_;

    std::deque<Frame> local_map_frames_;
    std::deque<Frame> global_map_frames_;

    bool has_new_local_map_ = false;
    bool has_new_global_map_ = false;

    CloudData::CloudPointTPtr local_map_ptr_;
    CloudData::CloudPointTPtr global_map_ptr_;
    CloudData::CloudPointTPtr result_cloud_ptr_;

    Frame current_frame_;
    
    float key_frame_distance_ = 2.0;
    int local_frame_num_ = 20;

};

}