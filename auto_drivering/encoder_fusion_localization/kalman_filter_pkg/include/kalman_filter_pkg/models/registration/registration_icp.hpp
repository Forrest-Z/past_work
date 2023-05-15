/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:48
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:37:29
 */
#pragma once

#include <pcl/registration/icp.h>
#include "kalman_filter_pkg/models/registration/registration_interface.hpp"
#include <yaml-cpp/yaml.h>


namespace KalmanFilter
{
class ICPRegistration : public RegistrationInterface
{
public:
    ICPRegistration(const YAML::Node &node);
    ICPRegistration(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter);

    bool SetInputTarget(const CloudData::CloudPointTPtr &input_target) override;
    bool ScanMatch(const CloudData::CloudPointTPtr &input_source,
                   const Eigen::Matrix4f &predict_pose,
                   CloudData::CloudPointTPtr &result_cloud_ptr,
                   Eigen::Matrix4f &result_pose) override;

private:
    bool SetRegistrationParam(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter);

private:
    pcl::IterativeClosestPoint<CloudData::PointT, CloudData::PointT>::Ptr icp_ptr_;
};
} // namespace lidar_localization
