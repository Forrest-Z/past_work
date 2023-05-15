/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:08:37
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:39:55
 */
#pragma once

#include <deque>
#include <Eigen/Dense>


namespace KalmanFilter
{
class VelocityData
{
public:
    struct LinearVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct AngularVelocity
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    double time = 0.0;
    LinearVelocity linear_vectority;
    AngularVelocity angular_vectory;

public:
    static bool SyncData(std::deque<VelocityData>& UnsyncedData, std::deque<VelocityData>& SyncedData, double sync_time);

    void TransformCoordinate(Eigen::Matrix4f transform_matrix);
    void NED2ENU(void);

};
    
} // namespace localization_ns
