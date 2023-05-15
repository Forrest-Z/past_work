/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-04 19:04:41
 * @LastEditors: luo
 * @LastEditTime: 2021-11-18 17:39:52
 */
#pragma once 

#include <string>
#include <Eigen/Dense>

namespace KalmanFilter
{
class PosVelData
{
public:
    double time = 0.0;

    Eigen::Vector3f pos = Eigen::Vector3f::Zero();
    Eigen::Vector3f vel = Eigen::Vector3f::Zero();
};
}