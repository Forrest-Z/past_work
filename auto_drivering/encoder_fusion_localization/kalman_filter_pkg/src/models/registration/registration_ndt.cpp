/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:19
 * @LastEditors: luo
 * @LastEditTime: 2021-12-27 12:41:51
 */
#include "kalman_filter_pkg/models/registration/registration_ndt.hpp"

// #include "glog/logging.h"

namespace KalmanFilter
{

NDTRegistration::NDTRegistration(const YAML::Node &node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointT, CloudData::PointT>())
{

    float res = node["res"].as<float>();
    float step_size = node["step_size"].as<float>();
    float trans_eps = node["trans_eps"].as<float>();
    int max_iter = node["max_iter"].as<int>();

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps, int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::PointT, CloudData::PointT>())
{

    SetRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::SetRegistrationParam(float res, float step_size, float trans_eps, int max_iter)
{
    ndt_ptr_->setResolution(res);
    ndt_ptr_->setStepSize(step_size);
    ndt_ptr_->setTransformationEpsilon(trans_eps);
    ndt_ptr_->setMaximumIterations(max_iter);
    // ndt_ptr_.setResolution(res);
    // ndt_ptr_.setStepSize(step_size);
    // ndt_ptr_.setTransformationEpsilon(trans_eps);
    // ndt_ptr_.setMaximumIterations(max_iter);

    std::cout << "NDT 的匹配参数为：" << std::endl
              << "res: " << res << ", "
              << "step_size: " << step_size << ", "
              << "trans_eps: " << trans_eps << ", "
              << "max_iter: " << max_iter
              << std::endl
              << std::endl;

    return true;
}

bool NDTRegistration::SetInputTarget(const CloudData::CloudPointTPtr &input_target)
{
    ndt_ptr_->setInputTarget(input_target);
    std::cout<< "NDTRegistration::SetInputTarget" << std::endl;
    // ndt_ptr_.setInputTarget(input_target);

    return true;
}

bool NDTRegistration::ScanMatch(const CloudData::CloudPointTPtr &input_source,
                                const Eigen::Matrix4f &predict_pose,
                                CloudData::CloudPointTPtr &result_cloud_ptr,
                                Eigen::Matrix4f &result_pose)
{
    // std::cout << "ScanMatch NDT 00" << std::endl;
    
    ndt_ptr_->setInputSource(input_source);
    // ndt_ptr_.setInputSource(input_source);
    // std::cout << "ScanMatch NDT 11" << std::endl;

    ndt_ptr_->align(*result_cloud_ptr, predict_pose);
    // ndt_ptr_.align(*result_cloud_ptr, predict_pose);
    //  std::cout << "ScanMatch NDT 44" << std::endl;
  
    // std::cout << "ScanMatch NDT 22" << std::endl;
    result_pose = ndt_ptr_->getFinalTransformation();
    // result_pose = ndt_ptr_.getFinalTransformation();
    // std::cout << "ScanMatch NDT 33" << std::endl;

    return true;
}

float NDTRegistration::GetFitnessScore() {
    return ndt_ptr_->getFitnessScore();
    // return ndt_ptr_.getFitnessScore();

}
}