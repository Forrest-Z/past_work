/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:48
 * @LastEditors: luo
 * @LastEditTime: 2021-12-27 12:41:36
 */
#include "kalman_filter_pkg/models/registration/registration_icp.hpp"

// #include "glog/logging.h"

namespace KalmanFilter
{
ICPRegistration::ICPRegistration(const YAML::Node& node)
  : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::PointT, CloudData::PointT>)
{
  float euclid_eps = node["euclid_eps"].as<float>();
  float max_correspond_dis = node["max_correspond_dis"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();
  SetRegistrationParam(euclid_eps, max_correspond_dis, trans_eps, max_iter);
}

ICPRegistration::ICPRegistration(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter)
  : icp_ptr_(new pcl::IterativeClosestPoint<CloudData::PointT, CloudData::PointT>)
{
  SetRegistrationParam(euclid_eps, max_correspond_dis, trans_eps, max_iter);
}

bool ICPRegistration::SetRegistrationParam(float euclid_eps, float max_correspond_dis, float trans_eps, int max_iter)
{
  icp_ptr_->setMaxCorrespondenceDistance(max_correspond_dis);
  icp_ptr_->setEuclideanFitnessEpsilon(euclid_eps);
  icp_ptr_->setTransformationEpsilon(trans_eps);
  icp_ptr_->setMaximumIterations(max_iter);

  std::cout << "ICP 的匹配参数为：" << std::endl
            << "euclid_eps: " << euclid_eps << ", "
            << "max_correspond_dis: " << max_correspond_dis << ", "
            << "trans_eps: " << trans_eps << ", "
            << "max_iter: " << max_iter << std::endl
            << std::endl;
  return true;
}

bool ICPRegistration::SetInputTarget(const CloudData::CloudPointTPtr& input_target)
{
  icp_ptr_->setInputTarget(input_target);

  return true;
}

bool ICPRegistration::ScanMatch(const CloudData::CloudPointTPtr& input_source, const Eigen::Matrix4f& predict_pose,
                                CloudData::CloudPointTPtr& result_cloud_ptr, Eigen::Matrix4f& result_pose)
{
  // std::cout << "ScanMatch 00" << std::endl;
  icp_ptr_->setInputSource(input_source);
  // std::cout << "ScanMatch 11" << std::endl;
  icp_ptr_->align(*result_cloud_ptr, predict_pose);
  // std::cout << "ScanMatch 22" << std::endl;
  result_pose = icp_ptr_->getFinalTransformation();
  // std::cout << "ScanMatch 33" << std::endl;
  return true;
}
}  