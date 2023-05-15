/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:04
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:36:58
 */
#pragma once

#include <yaml-cpp/yaml.h>
#include <pcl/filters/voxel_grid.h>
#include "kalman_filter_pkg/models/cloud_filter/cloud_filter_interface.hpp"

namespace KalmanFilter {
class VoxelFilter: public CloudFilterInterface {
  public:
    VoxelFilter(const YAML::Node& node);
    VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

    bool Filter(const CloudData::CloudPointTPtr& input_cloud_ptr, CloudData::CloudPointTPtr& filtered_cloud_ptr) override;

  private:
    bool SetFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  private:
    pcl::VoxelGrid<CloudData::PointT> voxel_filter_;
};
}