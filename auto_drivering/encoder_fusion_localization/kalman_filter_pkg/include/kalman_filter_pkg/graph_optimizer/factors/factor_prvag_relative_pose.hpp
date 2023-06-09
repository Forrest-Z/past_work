/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-23 14:49:44
 * @LastEditors: luo
 * @LastEditTime: 2022-01-17 17:45:37
 */
#pragma once

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace KalmanFilter
{
class FactorPRVAGRelativePose : public ceres::SizedCostFunction<6, 15, 15>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;

    FactorPRVAGRelativePose() {};

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
        //a pose i
        Eigen::Map<const Eigen::Vector3d>       pos_i(&parameters[0][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d>   log_ori_i(&parameters[0][INDEX_R]);
        const Sophus::SO3d                      ori_i(Sophus::SO3d::exp(log_ori_i));

        //b pose j
        Eigen::Map<const Eigen::Vector3d>       pos_j(&parameters[1][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d>   log_ori_j(&parameters[1][INDEX_R]);
        const Sophus::SO3d                      ori_j(Sophus::SO3d::exp(log_ori_j));

        //c parse measurement
        const Eigen::Vector3d       &pos_ij = m_.block<3, 1>(INDEX_P, 0);
        const Eigen::Vector3d       &log_ori_ij = m_.block<3, 1>(INDEX_R, 0);
        const Sophus::SO3d          ori_ij = Sophus::SO3d::exp(log_ori_ij);

        //d get square root of information matrix
        Eigen::Matrix<double, 6, 6> sqrt_info = Eigen::LLT<Eigen::Matrix<double, 6, 6>>(
        I_
        ).matrixL().transpose();


        //e compute residual
        Eigen::Map<Eigen::Matrix<double, 6, 1>> residual(residuals);
        residual.block(INDEX_P, 0, 3, 1) = ori_i.inverse() * (pos_j - pos_i) - pos_ij;
        residual.block(INDEX_R, 0, 3, 1) = (ori_i.inverse() * ori_j * ori_ij.inverse()).log();

        //f compute jacobians
        if(jacobians)
        {
            const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
            const Eigen::Matrix3d J_r_inv = JacobianRInv(residual.block(INDEX_R, 0, 3, 1));

            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
                jacobian_i.setZero();

                jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
                jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * (ori_ij * ori_j.inverse() * ori_i).matrix();

                jacobian_i = sqrt_info * jacobian_i;
                // std::cout << "relative_i_jacobian  = " << std::endl << jacobian_i << std::endl;
            }

            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 6, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
                jacobian_j.setZero();

                jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;
                jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv * ori_ij.matrix();

                jacobian_j = sqrt_info * jacobian_j;
                // std::cout << "relative_j_jacobian  = " << std::endl << jacobian_j << std::endl;
            }
        }

        residual = sqrt_info * residual;
        // std::cout << "sqrt_info  = " << std::endl << sqrt_info << std::endl;
        // std::cout << "residual  = " << std::endl << residual << std::endl;

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