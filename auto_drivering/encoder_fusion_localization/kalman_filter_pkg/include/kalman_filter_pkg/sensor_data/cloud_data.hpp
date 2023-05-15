/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:08
 * @LastEditors: luo
 * @LastEditTime: 2022-01-05 17:36:45
 */
#pragma once

#include <iostream>
#include <deque>
#include <mutex>

// #include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl_conversions/pcl_conversions.h>

namespace KalmanFilter
{
class CloudData
{
public:
    using PointT = pcl::PointXYZ;
    using CloudPointT = pcl::PointCloud<PointT>;
    using CloudPointTPtr = CloudPointT::Ptr;

public:
    CloudData()
    :cloud_ptr(new CloudPointT())
    {
        
    }
    
public:
    double time = 0.0;
    CloudPointTPtr cloud_ptr;
};
}