/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-22 10:20:34
 * @LastEditors: luo
 * @LastEditTime: 2022-01-17 10:52:52
 */

#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace KalmanFilter
{
class PreIntegrator
{
public:
    bool IsInited(void) const 
    { 
        return is_inited_;
    }

    double GetTime(void) const { return time_;}


protected:
    PreIntegrator(){}

    bool is_inited_ = false;

    double time_;

};
}