/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:04:58
 * @LastEditors: luo
 * @LastEditTime: 2021-12-22 11:09:30
 */
#pragma once

#include <iostream>
#include <deque>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>

namespace KalmanFilter
{
class IMUData
{
public:
    struct LinearAcc
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    
    };

    struct AngleVel
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    }; 

    struct AccelBias
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };  

    struct GyroBias
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };    
       
    

    class Orientation
    {
    public:
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        double w = 0.0;


    public:
        void Normlize()
        {
            double norm = sqrt(pow(x, 2.0) + pow(y, 2.0) + pow(z, 2.0) + pow(w, 2.0));
            x /= norm;
            y /= norm;
            z /= norm;
            w /= norm;
        }
    };   

    double time = 0.0;

    LinearAcc linear_acc;
    AngleVel angular_vel;
    Orientation orientation;

    AccelBias accel_bias;
    GyroBias gyro_bias;

public:
    //四元数------>旋转矩阵
    Eigen::Matrix3f GetOrientationMatrix();
    static bool SyncData(std::deque<IMUData> & UnsyncedData, std::deque<IMUData> &SyncedData, double sysnc_time);

};
}