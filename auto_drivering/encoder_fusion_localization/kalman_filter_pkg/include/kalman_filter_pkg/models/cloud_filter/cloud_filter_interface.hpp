/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 19:56:09
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:37:12
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include "kalman_filter_pkg/sensor_data/cloud_data.hpp"
// #include <yaml-cpp/yaml.h>


namespace KalmanFilter {
class CloudFilterInterface {
  public:
    virtual ~CloudFilterInterface() = default;

    virtual bool Filter(const CloudData::CloudPointTPtr& input_cloud_ptr, CloudData::CloudPointTPtr& filtered_cloud_ptr) = 0;
};
}
