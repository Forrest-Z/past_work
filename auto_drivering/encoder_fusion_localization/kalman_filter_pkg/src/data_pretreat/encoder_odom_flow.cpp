/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2022-01-13 21:04:46
 * @LastEditors: luo
 * @LastEditTime: 2022-01-14 14:17:18
 */
#include "kalman_filter_pkg/data_pretreat/encoder_odom_flow.hpp"

#include <geometry_msgs/Pose2D.h>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>
#include <deque>


boost::array<double, 36UL> ODOM_POSE_COVARIANCE = {1e-3, 0, 0, 0, 0, 0, 
                        0, 1e-3, 0, 0, 0, 0,
                        0, 0, 1e6, 0, 0, 0,
                        0, 0, 0, 1e6, 0, 0,
                        0, 0, 0, 0, 1e6, 0,
                        0, 0, 0, 0, 0, 1e3};

boost::array<double, 36UL> ODOM_POSE_COVARIANCE2 = {1e-9, 0, 0, 0, 0, 0, 
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9};
 
boost::array<double, 36UL> ODOM_TWIST_COVARIANCE = {1e-3, 0, 0, 0, 0, 0, 
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3};
boost::array<double, 36UL> ODOM_TWIST_COVARIANCE2 = {1e-9, 0, 0, 0, 0, 0, 
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9};


namespace KalmanFilter
{

EncoderOdomFlow::EncoderOdomFlow()
{

}

EncoderOdomFlow::~EncoderOdomFlow()
{

}

bool EncoderOdomFlow::Run()
{

}

bool EncoderOdomFlow::PublishData()
{

}
}