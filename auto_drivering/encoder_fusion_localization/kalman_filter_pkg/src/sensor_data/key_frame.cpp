/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 17:45:38
 * @LastEditors: luo
 * @LastEditTime: 2021-12-21 17:59:38
 */

#include "kalman_filter_pkg/sensor_data/key_frame.hpp"

namespace KalmanFilter
{

Eigen::Quaternionf KeyFrame::GetQuaternion() const
{
    Eigen::Quaternionf q;

    q = pose.block<3, 3>(0, 0);

    return q;
}


Eigen::Vector3f KeyFrame::GetTranslation() const
{
    Eigen::Vector3f t = pose.block<3, 1>(0, 3);

    return t;

}

}