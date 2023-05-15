/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:33
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:37:28
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"

namespace KalmanFilter
{
class RegistrationInterface
{
public:
  virtual ~RegistrationInterface() = default;

  virtual bool SetInputTarget(const CloudData::CloudPointTPtr &input_target) = 0;
  virtual bool ScanMatch(const CloudData::CloudPointTPtr &input_source,
                         const Eigen::Matrix4f &predict_pose,
                         CloudData::CloudPointTPtr &result_cloud_ptr,
                         Eigen::Matrix4f &result_pose) = 0;
};
} // namespace lidar_localization
