/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:04
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:43:27
 */

#include "kalman_filter_pkg/models/cloud_filter/voxel_filter.hpp"
#include <yaml-cpp/yaml.h>


// #include "glog/logging.h"

namespace KalmanFilter {

VoxelFilter::VoxelFilter(const YAML::Node& node) {
    float leaf_size_x = node["leaf_size"][0].as<float>();
    float leaf_size_y = node["leaf_size"][1].as<float>();
    float leaf_size_z = node["leaf_size"][2].as<float>();
    
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    SetFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z) {
    voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);

    std::cout << "Voxel Filter 的参数为：" << std::endl
              << leaf_size_x << ", "
              << leaf_size_y << ", "
              << leaf_size_z 
              << std::endl << std::endl;

    return true;
}

bool VoxelFilter::Filter(const CloudData::CloudPointTPtr& input_cloud_ptr, CloudData::CloudPointTPtr& filtered_cloud_ptr) {
    // std::cout << "Filter 00" << std::endl;
    voxel_filter_.setInputCloud(input_cloud_ptr);
    // std::cout << "Filter 11" << std::endl;
    CloudData::CloudPointTPtr cloud_points(new CloudData::CloudPointT());
    voxel_filter_.filter(*cloud_points);
    // std::cout << "Filter 22" << std::endl;
    filtered_cloud_ptr = cloud_points;
    // std::cout << "Filter 33" << std::endl;

    return true;
}
} 