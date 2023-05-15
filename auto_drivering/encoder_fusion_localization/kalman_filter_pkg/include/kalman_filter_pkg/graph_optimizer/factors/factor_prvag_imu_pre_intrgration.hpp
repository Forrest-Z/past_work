/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-22 14:40:29
 * @LastEditors: luo
 * @LastEditTime: 2022-01-18 15:08:56
 */
#pragma once

#include <ceres/ceres.h>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>

#include <sophus/so3.hpp>

#include "glog/logging.h"

namespace KalmanFilter
{
class FactorPRVAGIMUPreIntrgration : public ceres::SizedCostFunction<15, 15, 15>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

    FactorPRVAGIMUPreIntrgration(void ) {};

    void SetT(const double& T)
    {
        T_ = T;
    }

    void SetGravity(const Eigen::Vector3d& g)
    {
        g_ = g;
    }

    void SetMeasurement(const Eigen::VectorXd& m)
    {
        m_ = m;
    }

    void SetInformation(const Eigen::MatrixXd& I)
    {
        I_ = I;
    }

    void SetJacobian(const Eigen::MatrixXd& J)
    {
        J_ = J;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const 
    {
        //a. pose i
        Eigen::Map<const Eigen::Vector3d>       pos_i(&parameters[0][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d>   log_ori_i(&parameters[0][INDEX_R]);
        const Sophus::SO3d                      ori_i = Sophus::SO3d::exp(log_ori_i);
        Eigen::Map<const Eigen::Vector3d>       vel_i(&parameters[0][INDEX_V]);
        Eigen::Map<const Eigen::Vector3d>       b_a_i(&parameters[0][INDEX_A]);
        Eigen::Map<const Eigen::Vector3d>       b_g_i(&parameters[0][INDEX_G]);

        //pose j
        Eigen::Map<const Eigen::Vector3d>       pos_j(&parameters[1][INDEX_P]);
        Eigen::Map<const Eigen::Vector3d>   log_ori_j(&parameters[1][INDEX_R]);
        const Sophus::SO3d                      ori_j = Sophus::SO3d::exp(log_ori_j);
        Eigen::Map<const Eigen::Vector3d>       vel_j(&parameters[1][INDEX_V]);
        Eigen::Map<const Eigen::Vector3d>       b_a_j(&parameters[1][INDEX_A]);
        Eigen::Map<const Eigen::Vector3d>       b_g_j(&parameters[1][INDEX_G]);

        //parse measurement
        Eigen::Vector3d alpha_ij = m_.block<3, 1>(INDEX_P, 0);
        Eigen::Vector3d theta_ij = m_.block<3, 1>(INDEX_R, 0);
        Eigen::Vector3d  beta_ij = m_.block<3, 1>(INDEX_V, 0);

        // //bias变化量
        // Eigen::Vector3d dba = b_a_i - m_.block<3, 1>(INDEX_A, 0);
        // Eigen::Vector3d dbg = b_g_i - m_.block<3, 1>(INDEX_G, 0);

        // //根据变化量使用jacobian重新计算预积分
        // Eigen::Matrix3d J_q_bg = J_.block<3, 3>(INDEX_R, INDEX_G);
        // Eigen::Matrix3d J_v_ba = J_.block<3, 3>(INDEX_V, INDEX_A);
        // Eigen::Matrix3d J_v_bg = J_.block<3, 3>(INDEX_V, INDEX_G);
        // Eigen::Matrix3d J_p_ba = J_.block<3, 3>(INDEX_P, INDEX_A);
        // Eigen::Matrix3d J_p_bg = J_.block<3, 3>(INDEX_P, INDEX_G);

        // Sophus::SO3d ori_ij = Sophus::SO3d::exp(theta_ij) * Sophus::SO3d::exp(J_q_bg * dbg);

        // alpha_ij = alpha_ij + J_p_ba * dba + J_p_bg * dbg;
        // beta_ij = beta_ij + J_v_ba * dba + J_v_bg * dbg;

        Eigen::LLT<Eigen::Matrix<double, 15, 15> > LowerI(I_);
        //get square root of information matrix
        Eigen::Matrix<double, 15, 15> sqrt_info = LowerI.matrixL().transpose();

        std::cout << "------imu_pre_residual ------000000000000---sqrt_info---- = " << std::endl << sqrt_info << std::endl; 

        // const Eigen::Matrix3d oriRT_i = ori_i.inverse().matrix();
        // Eigen::Map<Eigen::Matrix<double, 15, 1> > resid(residuals);
        // resid.block<3, 1>(INDEX_P, 0) = oriRT_i * (pos_j - pos_i - vel_i * T_ + 0.5 * g_ * T_ * T_) - alpha_ij;
        // resid.block<3, 1>(INDEX_R, 0) = (ori_ij.inverse() * (ori_ij.inverse() * ori_j)).log();
        // resid.block<3, 1>(INDEX_V, 0) = oriRT_i * (vel_j - vel_i + g_ * T_) - beta_ij;
        // resid.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
        // resid.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;


        // compute redsidual
        Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
        residual.block<3, 1>(INDEX_P, 0) = ori_i.inverse() * (pos_j - pos_i - (vel_i - 0.5* g_ * T_) * T_) - alpha_ij;
        residual.block<3, 1>(INDEX_R, 0) = (Sophus::SO3d::exp(theta_ij).inverse() * ori_i.inverse() * ori_j).log();
        residual.block<3, 1>(INDEX_V, 0) = ori_i.inverse() * (vel_j - vel_i + g_ * T_) - beta_ij;
        residual.block<3, 1>(INDEX_A, 0) = b_a_j - b_a_i;
        residual.block<3, 1>(INDEX_G, 0) = b_g_j - b_g_i;

        // std::cout << std::endl;
        // std::cout << std::endl;
        // std::cout << std::endl;
        // std::cout << std::endl;
        // std::cout << std::endl;
        // std::cout << "pos_i = " << pos_i << std::endl;
        // std::cout << "pos_j = " << pos_j << std::endl;
        // std::cout << "vel_i = " << vel_i << std::endl;
        // std::cout << "g_ = " << g_ << std::endl;
        // std::cout << "T_ = " << T_ << std::endl;
        // std::cout << "------imu_pre_residual ------000000000000---ori_i---- = " << std::endl << log_ori_i << std::endl; 
        // // std::cout << "------imu_pre_residual ------000000000000---ori_i.inv---- = " << std::endl << ori_i.inverse() << std::endl; 
        // std::cout << "------imu_pre_residual ------000000000000---mid---- = " << std::endl << (pos_j - pos_i - (vel_i - 0.5* g_ * T_)*T_) << std::endl; 
        // std::cout << "------imu_pre_residual ------000000000000---alpha_ij---- = " << std::endl << alpha_ij << std::endl; 
        // std::cout << "------imu_pre_residual ------000000000000---jacobians---- = " << std::endl << residual << std::endl;

        // if(jacobians)
        // {
        //     Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor> > jacobian_i(jacobians[0]);
        //     Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor> > jacobian_j(jacobians[1]);
        //     jacobian_i.setZero();
        //     jacobian_j.setZero();

        //     const Eigen::Vector3d deltaR = resid.block<3, 1>(INDEX_R, 0);
        //     const Eigen::Matrix3d J_r_inv = JacobianRInv(deltaR);

        //     if(jacobians[0])
        //     {
        //         //a. residual, position
        //         jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -oriRT_i;
        //         jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(ori_i.inverse() * 
        //                                             (pos_j - pos_i -vel_i * T_ + 0.50 * g_ * T_ *T_)).matrix();
        //         jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -oriRT_i * T_;
        //         jacobian_i.block<3, 3>(INDEX_P, INDEX_A) = -J_.block<3, 3>(INDEX_P, INDEX_A);
        //         jacobian_i.block<3, 3>(INDEX_P, INDEX_G) = -J_.block<3, 3>(INDEX_P, INDEX_G);

        //         //b. residual orientation
        //         jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * ((ori_j.inverse() * ori_i).matrix());
        //         jacobian_i.block<3, 3>(INDEX_R, INDEX_G) = 
        //                     -J_r_inv *
        //                     Sophus::SO3d::exp(resid.block<3, 1>(INDEX_R, 0)).matrix().inverse() *
        //                     JacobianR(J_.block<3, 3>(INDEX_R, INDEX_G) * (b_g_i - m_.block<3, 1>(INDEX_G, 0))) *
        //                     J_.block<3, 3>(INDEX_R, INDEX_G);
                
        //         //b. residual, velocity
        //         jacobian_i.block<3, 3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(ori_i.inverse() * (vel_j - vel_i + g_ * T_));
        //         jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -oriRT_i;
        //         jacobian_i.block<3, 3>(INDEX_V, INDEX_A) = -J_.block<3, 3>(INDEX_V, INDEX_A);
        //         jacobian_i.block<3, 3>(INDEX_V, INDEX_G) = -J_.block<3, 3>(INDEX_V, INDEX_G);

        //         //d. residual bias accel
        //         jacobian_i.block<3, 3>(INDEX_A, INDEX_A) = -Eigen::Matrix3d::Identity();
        //         jacobian_i.block<3, 3>(INDEX_G, INDEX_G) = -Eigen::Matrix3d::Identity();

        //     }


        //     if(jacobians[1])
        //     {
        //         //a. residual position
        //         jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = oriRT_i;

        //         //b. residual orientation
        //         jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv;

        //         //c. residual velocity
        //         jacobian_j.block<3, 3>(INDEX_V, INDEX_V) = oriRT_i;

        //         //d. residual bias accel
        //         jacobian_j.block<3, 3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();
        //         jacobian_j.block<3, 3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();
        //     }

        //     jacobian_i = sqrt_info * jacobian_i;
        //     jacobian_j = sqrt_info * jacobian_j;

        // }

        // resid = sqrt_info * resid;

        // return true;

        //compute jacobians
        if(jacobians)
        {
            const Eigen::Matrix3d R_i_inv = ori_i.inverse().matrix();
            const Eigen::Matrix3d J_r_inv = JacobianRInv(residual.block(INDEX_R, 0, 3, 1));


            // std::cout << "------imu_pre_residual ------11111111---jacobians---- " << std::endl; 
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_i(jacobians[0]);
                jacobian_i.setZero();

                //a residual. position
                jacobian_i.block<3, 3>(INDEX_P, INDEX_P) = -R_i_inv;
                jacobian_i.block<3, 3>(INDEX_P, INDEX_R) = Sophus::SO3d::hat(
                                ori_i.inverse() * (pos_j - pos_i - ( vel_i - 0.50 * g_ * T_) * T_));
                jacobian_i.block<3, 3>(INDEX_P, INDEX_V) = -T_ * R_i_inv;
                jacobian_i.block<3, 3>(INDEX_P, INDEX_A) = -J_.block<3, 3>(INDEX_P, INDEX_A);
                jacobian_i.block<3, 3>(INDEX_P, INDEX_G) = -J_.block<3, 3>(INDEX_P, INDEX_G);

                // std::cout << "------imu_pre_residual ------jacobian_i---pppppppppppppppppp---- " << std::endl ;
                // std::cout << "pos_i = " << pos_i << std::endl;
                // std::cout << "pos_j = " << pos_j << std::endl;
                // std::cout << "vel_i = " << vel_i << std::endl;
                // std::cout << "g_ = " << g_ << std::endl;

                //             << jacobian_i.block<3, 3>(INDEX_P, INDEX_V)
                //             << jacobian_i.block<3, 3>(INDEX_P, INDEX_A)
                //             << jacobian_i.block<3, 3>(INDEX_P, INDEX_G)
                //             << std::endl;


                //b residual, orientation
                jacobian_i.block<3, 3>(INDEX_R, INDEX_R) = -J_r_inv * (ori_j.inverse() * ori_i).matrix();
                jacobian_i.block<3, 3>(INDEX_R, INDEX_G) = -J_r_inv * (
                            Sophus::SO3d::exp(residual.block<3, 1>(INDEX_P, 0)
                                                            ).matrix().inverse() * J_.block<3, 3>(INDEX_R, INDEX_G));


                //c residual velocity
                jacobian_i.block<3, 3>(INDEX_V, INDEX_R) = Sophus::SO3d::hat(
                                ori_i.inverse() * (vel_j - vel_i + g_ * T_));
                jacobian_i.block<3, 3>(INDEX_V, INDEX_V) = -R_i_inv;
                jacobian_i.block<3, 3>(INDEX_V, INDEX_A) = -J_.block<3, 3>(INDEX_V, INDEX_A);
                jacobian_i.block<3, 3>(INDEX_V, INDEX_G) = -J_.block<3, 3>(INDEX_V, INDEX_G);

                //d residual bias accel
                jacobian_i.block<3, 3>(INDEX_A, INDEX_A) = -Eigen::Matrix3d::Identity();

                //e residual bias gyro
                jacobian_i.block<3, 3>(INDEX_G, INDEX_G) = -Eigen::Matrix3d::Identity();
                // std::cout << "------imu_pre_residual ------jacobian_i---rrr----= " << std::endl 
                            // << jacobian_i.block<3, 3>(INDEX_R, INDEX_R) 
                            // << jacobian_i.block<3, 3>(INDEX_R, INDEX_G)
                            // << std::endl;
            std::cout << "------imu_pre_residual ------33333333333---jacobian_i----= " << std::endl << jacobian_i << std::endl; 

                jacobian_i = sqrt_info * jacobian_i;
            }
            
            if(jacobians[1])
            {
                Eigen::Map<Eigen::Matrix<double, 15, 15, Eigen::RowMajor>> jacobian_j(jacobians[1]);
                jacobian_j.setZero();

                //a resudual, position
                jacobian_j.block<3, 3>(INDEX_P, INDEX_P) = R_i_inv;

                //b residual, orientation
                jacobian_j.block<3, 3>(INDEX_R, INDEX_R) = J_r_inv;

                //c residual velocity
                jacobian_j.block<3, 3>(INDEX_V, INDEX_V) = R_i_inv;

                //d residual bias accel
                jacobian_j.block<3, 3>(INDEX_A, INDEX_A) = Eigen::Matrix3d::Identity();

                //e residual bias gyro
                jacobian_j.block<3, 3>(INDEX_G, INDEX_G) = Eigen::Matrix3d::Identity();

                std::cout << "------imu_pre_residual --444444444444----jacobian_j-------= " << std::endl << jacobian_j << std::endl;

                jacobian_j = sqrt_info * jacobian_j;
            }
        // std::cout << "------imu_pre_residual ------000000000000---jacobian_j----= " << std::endl << jacobian_j << std::endl ;

        }
        std::cout << "--------------------imu_pre_residual ---5555555----sqrt_info------= " << std::endl << sqrt_info << std::endl;
        // correct residual by square root of information matrix:
        residual = sqrt_info * residual;

        std::cout << "--------------------imu_pre_residual ---66666666666---residual-------= " << std::endl << residual << std::endl;

        return true;
    }

private:
  static Eigen::Matrix3d JacobianRInv(const Eigen::Vector3d &w) {
      Eigen::Matrix3d J_r_inv = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if ( theta > 1e-5 ) {
        Eigen::Vector3d k = w.normalized();
        Eigen::Matrix3d K = Sophus::SO3d::hat(k);
        J_r_inv = J_r_inv 
                    + 0.5 * K
                    + (1.0 - (1.0 + std::cos(theta)) * theta / (2.0 * std::sin(theta))) * K * K;

        // Eigen::Vector3d a = w.normalized();
        // Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
        // double theta_half = 0.5 * theta;
        // double cot_theta = 1.0 / tan(theta_half);
        //  J_r_inv = theta_half * cot_theta * J_r_inv + 
        //             (1.0 - theta_half * cot_theta) * a * a.transpose() + theta_half * a_hat;
      }

      return J_r_inv;
  }

  static Eigen::Matrix3d JacobianR(const Eigen::Vector3d &w)
  {
      Eigen::Matrix3d J_r = Eigen::Matrix3d::Identity();

      double theta = w.norm();

      if(theta > 1e-5)
      {
          Eigen::Vector3d a = w.normalized();
          Eigen::Matrix3d a_hat = Sophus::SO3d::hat(a);
        J_r = sin(theta) / theta * Eigen::Matrix3d::Identity() + (1.0 - sin(theta) / theta) * a * a.transpose()
                       - (1.0 - cos(theta)) / theta * a_hat;

        //   J_r = sin(theta) / theta * Eigen::Matrix3d::Identity() + (1.0 - sin(theta)) / theta ) * a * a.transpose() 
        //                 - (1.0 - cos(theta)) / theta * a_hat;
      }

      return J_r;
  }


private:

    double T_ = 0.0;

    Eigen::Vector3d g_ = Eigen::Vector3d::Zero();

    Eigen::VectorXd m_;

    Eigen::MatrixXd I_;

    Eigen::MatrixXd J_;
};
    
} // namespace KalmanFilter
