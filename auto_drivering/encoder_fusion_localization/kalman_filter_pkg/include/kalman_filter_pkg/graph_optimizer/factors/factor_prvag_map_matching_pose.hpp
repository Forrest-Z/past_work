/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-23 10:56:16
 * @LastEditors: luo
 * @LastEditTime: 2022-01-17 17:45:24
 */
#pragma once

#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace KalmanFilter
{
class FactorPRVAGMapMatchingPose : public ceres::SizedCostFunction<6, 15>
{
public: 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int INDEX_P = 0;
    static const int INDEX_R = 3;

    FactorPRVAGMapMatchingPose(){};

    void SetMeasurement(const Eigen::VectorXd& m)
    {
        m_ = m;
    }

    void SetInformation(const Eigen::MatrixXd& I)
    {
        I_ = I;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const 
    {
        //pose
        Eigen::Map<const Eigen::Vector3d>       pos(&parameters[0][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d>   log_ori(&parameters[0][INDEX_R]);
        const Sophus::SO3d                      ori = Sophus::SO3d::exp(log_ori);

        //parse measurement
        const Eigen::Vector3d       &pos_prior = m_.block<3, 1>(INDEX_P, 0);
        const Eigen::Vector3d   &log_ori_prior = m_.block<3, 1>(INDEX_R, 0);
        const Sophus::SO3d           ori_prior = Sophus::SO3d::exp(log_ori_prior);

        //get squate root of information matrix
        Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(I_).matrixL().transpose();

        //compute residual
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.block(INDEX_P, 0, 3, 1) = pos - pos_prior;
        residual.block(INDEX_R, 0, 3, 1) = (ori * ori_prior.inverse()).log();

        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_prior(jacobians[0]);
                jacobian_prior.setZero();

                jacobian_prior.block<3, 3>(INDEX_P, INDEX_P) = Eigen::Matrix3d::Identity();
                jacobian_prior.block<3, 3>(INDEX_R, INDEX_R) = JacobianRInv(
                                                residual.block(INDEX_R, 0, 3, 1)) * ori_prior.matrix();
                jacobian_prior = sqrt_info * jacobian_prior;
            }
        }

        residual = sqrt_info * residual;

        return true;
    }

private:
    static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d& w)
    {
        Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

        double theta = w.norm();

        if(theta > 1e-5)
        {
            Eigen::Vector3d k = w.normalized();
            Eigen::Matrix3d K = Sophus::SO3d::hat(k);

            J_r_inv = J_r_inv + 0.5 * K + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K; 
        }

        return J_r_inv;
    }



private:


    Eigen::VectorXd m_;
    Eigen::MatrixXd I_;
};
}