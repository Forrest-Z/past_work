/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:19
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 18:59:17
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include "kalman_filter_pkg/models/registration/registration_interface.hpp"
#include <yaml-cpp/yaml.h>

namespace KalmanFilter
{
class NDTRegistration : public RegistrationInterface
{
public:
  NDTRegistration(const YAML::Node &node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool SetInputTarget(const CloudData::CloudPointTPtr &input_target) override;
  bool ScanMatch(const CloudData::CloudPointTPtr &input_source,
                 const Eigen::Matrix4f &predict_pose,
                 CloudData::CloudPointTPtr &result_cloud_ptr,
                 Eigen::Matrix4f &result_pose) override;

  float GetFitnessScore();
private:
  bool SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter);

private:
  pcl::NormalDistributionsTransform<CloudData::PointT, CloudData::PointT>::Ptr ndt_ptr_;
  // pcl::NormalDistributionsTransform<CloudData::PointT, CloudData::PointT> ndt_ptr_;

};
} 
