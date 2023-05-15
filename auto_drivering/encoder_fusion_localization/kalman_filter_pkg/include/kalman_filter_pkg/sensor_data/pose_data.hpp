/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:08:26
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 16:09:09
 */
#pragma once

#include <deque>
#include <Eigen/Dense>

namespace KalmanFilter
{
class PoseData
{
public:
    double time = 0.0;

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
    
    struct {
        Eigen::Vector3f v = Eigen::Vector3f::Zero();
        Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } Vel;

public:
    Eigen::Quaternionf GetQuaternion();
    static bool SyncData(std::deque<PoseData> & UnsyncedData, std::deque<PoseData> &SyncedData, double sync_time);
};
}