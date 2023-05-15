/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-01-17 11:00:08
 * @LastEditors: luo
 * @LastEditTime: 2022-01-18 15:03:42
 */
#pragma once

#include <iostream>

#include "kalman_filter_pkg/imu_pre_integrator/pre_integrator.hpp"
#include "kalman_filter_pkg/sensor_data/velocity_data.hpp"
#include "kalman_filter_pkg/graph_optimizer/params/param_pravg.hpp"

#include <g2o/
#include <sophus/so3.hpp>

namespace KalmanFilter
{
class OdomPreIntrgrator : public PreIntegrator
{
public:
    static const int DIM_STATE = 6;

    typedef Eigen::Matrix<double, DIM_STATE, DIM_STATE> MatrixP;

    struct OdomPreIntrgration
    {
        //a. measurement
        //a. 1 relative translation
        Eigen::Vector3d alpha_ij_;

        //a.2 relative orientation
        Sophus::SO3d theta_ij_;

        //b. information
        MatrixP P_;

        
    };

};
    
}