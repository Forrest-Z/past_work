/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-01-17 17:38:08
 * @LastEditors: luo
 * @LastEditTime: 2022-01-17 17:44:51
 */
#pragma once

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include <glog/logging.h>

namespace KalmanFilter
{
class FactorPRVAGEncoderPreIntrgration : public ceres::SizedCostFunction<6, 15, 15>
{

};

}