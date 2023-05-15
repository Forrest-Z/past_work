/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:05:17
 * @LastEditors: luo
 * @LastEditTime: 2021-11-22 14:39:07
 */
#pragma once

#include <iostream>
#include <deque>
#include "/home/luo/work/filter_ws/src/kalman_filter_pkg/third_party/GeographicLib/include/Geocentric/LocalCartesian.hpp"
#include <Eigen/Dense>

namespace KalmanFilter
{
class GNSSData
{
public:
    double time = 0.0;
    double longitude = 0.0;
    double latitude = 0.0;
    double altitude = 0.0;
    double local_E = 0.0;
    double local_N = 0.0;
    double local_U = 0.0;

    Eigen::Vector3f vel{0.0, 0.0, 0.0};

    int status = 0;
    int service = 0;

private:
    static GeographicLib::LocalCartesian geo_converter;
    static bool origin_position_inited;


public:
    void InitOriginPosition();
    void UpdateXYZ();

    static bool SyncData(std::deque<GNSSData>& UnsyncedData, std::deque<GNSSData> &SyncedData, double sync_time);
};

}