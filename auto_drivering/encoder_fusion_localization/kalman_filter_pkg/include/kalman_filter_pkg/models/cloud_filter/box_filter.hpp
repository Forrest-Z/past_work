/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-11 11:41:49
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:36:35
 */

#pragma once

#include <yaml-cpp/yaml.h>
#include <pcl/filters/crop_box.h>
#include "kalman_filter_pkg/models/cloud_filter/cloud_filter_interface.hpp"

namespace KalmanFilter {
class BoxFilter: public CloudFilterInterface {
  public:
    BoxFilter(const YAML::Node& node);
    ~BoxFilter();

    bool Filter(const CloudData::CloudPointTPtr& input_cloud_ptr, CloudData::CloudPointTPtr& filtered_cloud_ptr) override;

    void SetSize(std::vector<float> size);
    void SetOrigin(std::vector<float> origin);
    std::vector<float> GetEdge();

  private:
    void CalculateEdge();

  private:
    pcl::CropBox<CloudData::PointT> pcl_box_filter_;

    std::vector<float> origin_;
    std::vector<float> size_;
    std::vector<float> edge_;
};
}