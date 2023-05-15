/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 17:45:49
 * @LastEditors: luo
 * @LastEditTime: 2021-12-21 17:57:57
 */
#pragma once

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sophus/so3.hpp>

namespace KalmanFilter
{
class KeyFrame
{
public:
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

    double time = 0.0;

    unsigned int index = 0;

    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();

    struct{
        Eigen::Vector3f v = Eigen::Vector3f::Zero();
        Eigen::Vector3f w = Eigen::Vector3f::Zero();
    } vel;

    struct{
        Eigen::Vector3f accel = Eigen::Vector3f::Zero();
        Eigen::Vector3f gyro = Eigen::Vector3f::Zero();
    } bias;

    KeyFrame() {}

    explicit KeyFrame(const int param_index, const double &T, const double *pravg)
    {
        time = T;

        index = param_index;

        Eigen::Map<const Eigen::Vector3d>       pos(pravg + INDEX_P);
        Eigen::Map<const Eigen::Vector3d>       log_ori(pravg + INDEX_R);
        Eigen::Map<const Eigen::Vector3d>       v(pravg + INDEX_V);
        Eigen::Map<const Eigen::Vector3d>       b_a(pravg + INDEX_A);
        Eigen::Map<const Eigen::Vector3d>       b_g(pravg + INDEX_G);

        pose.block<3, 1>(0, 3) = pos.cast<float>();
        pose.block<3, 3>(0, 0) = Sophus::SO3d::exp(log_ori).matrix().cast<float>();

        vel.v = v.cast<float>();

        bias.accel = b_a.cast<float>();
        bias.gyro = b_g.cast<float>();
    }

    Eigen::Quaternionf GetQuaternion() const;
    Eigen::Vector3f GetTranslation() const;

};
}