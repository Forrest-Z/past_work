//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/factor/ImuFactor.h"

namespace gp_lio
{

ImuFactor::ImuFactor(gp_lio::ImuPreintegration *imu_preintegration) : imu_preintegration_(imu_preintegration)
{
}

bool ImuFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    // load parameters to states
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);
    Eigen::Vector3d Vi(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Vector3d Bai(parameters[1][3], parameters[1][4], parameters[1][5]);
    Eigen::Vector3d Bgi(parameters[1][6], parameters[1][7], parameters[1][8]);

    Eigen::Vector3d Pj(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond Qj(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    Eigen::Vector3d Vj(parameters[3][0], parameters[3][1], parameters[3][2]);
    Eigen::Vector3d Baj(parameters[3][3], parameters[3][4], parameters[3][5]);
    Eigen::Vector3d Bgj(parameters[3][6], parameters[3][7], parameters[3][8]);

    // compute residual
    Eigen::Map<Eigen::Matrix<double, 15, 1>> residual(residuals);
    residual = imu_preintegration_->Evaluate(Pi, Qi, Vi, Bai, Bgi, Pj, Qj, Vj, Baj, Bgj);
    // ROS_DEBUG_STREAM("residual 1: " << residual.transpose());
    Eigen::Matrix<double, 15, 15> sqrt_info_matrix = Eigen::LLT<Eigen::Matrix<double, 15, 15>>(imu_preintegration_->covariance_.inverse()).matrixL().transpose();

    residual = sqrt_info_matrix * residual;
    // ROS_DEBUG_STREAM("residual 2: " << residual.transpose());
    // update jacobians_
    if (jacobians)
    {
        double sum_dt = imu_preintegration_->sum_dt_;
        Eigen::Matrix3d dp_dba = imu_preintegration_->jacobian_.template block<3, 3>(O_P, O_BA);
        Eigen::Matrix3d dp_dbg = imu_preintegration_->jacobian_.template block<3, 3>(O_P, O_BG);

        Eigen::Matrix3d dq_dbg = imu_preintegration_->jacobian_.template block<3, 3>(O_R, O_BG);

        Eigen::Matrix3d dv_dba = imu_preintegration_->jacobian_.template block<3, 3>(O_V, O_BA);
        Eigen::Matrix3d dv_dbg = imu_preintegration_->jacobian_.template block<3, 3>(O_V, O_BG);

        if (imu_preintegration_->jacobian_.maxCoeff() > 1e8 || imu_preintegration_->jacobian_.minCoeff() < -1e8)
        {
            ROS_WARN("numerical unstable in preintegration");
            //std::cout << pre_integration->jacobian << std::endl;
            ///                ROS_BREAK();
        }

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            jacobian_pose_i.setZero();

            jacobian_pose_i.block<3, 3>(O_P, O_P) = -Qi.inverse().toRotationMatrix();
            jacobian_pose_i.block<3, 3>(O_P, O_R) = Utility::skewSymmetric(
                Qi.inverse() * (0.5 * GRAVITY * sum_dt * sum_dt + Pj - Pi - Vi * sum_dt));

#if 0
                jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Qj.inverse() * Qi).toRotationMatrix();
#else
            Eigen::Quaterniond corrected_delta_q =
                imu_preintegration_->delta_q_ * Utility::deltaQ(dq_dbg * (Bgi - imu_preintegration_->linearized_bg_));
            jacobian_pose_i.block<3, 3>(O_R, O_R) = -(Utility::Qleft(Qj.inverse() * Qi) *
                                                      Utility::Qright(corrected_delta_q))
                                                         .bottomRightCorner<3, 3>();
#endif

            jacobian_pose_i.block<3, 3>(O_V, O_R) = Utility::skewSymmetric(Qi.inverse() * (GRAVITY * sum_dt + Vj - Vi));

            jacobian_pose_i = sqrt_info_matrix * jacobian_pose_i;

            if (jacobian_pose_i.maxCoeff() > 1e8 || jacobian_pose_i.minCoeff() < -1e8)
            {
                ROS_WARN("numerical unstable in preintegration");
                //std::cout << sqrt_info << std::endl;
                //ROS_BREAK();
            }
        }
        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_i(jacobians[1]);
            jacobian_speedbias_i.setZero();
            jacobian_speedbias_i.block<3, 3>(O_P, O_V - O_V) = -Qi.inverse().toRotationMatrix() * sum_dt;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BA - O_V) = -dp_dba;
            jacobian_speedbias_i.block<3, 3>(O_P, O_BG - O_V) = -dp_dbg;

#if 0
                jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -dq_dbg;
#else
            //Eigen::Quaterniond corrected_delta_q = pre_integration->delta_q * Utility::deltaQ(dq_dbg * (Bgi - pre_integration->linearized_bg));
            //jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) = -Utility::Qleft(Qj.inverse() * Qi * corrected_delta_q).bottomRightCorner<3, 3>() * dq_dbg;
            jacobian_speedbias_i.block<3, 3>(O_R, O_BG - O_V) =
                -Utility::Qleft(Qj.inverse() * Qi * imu_preintegration_->delta_q_).bottomRightCorner<3, 3>() *
                dq_dbg;
#endif

            jacobian_speedbias_i.block<3, 3>(O_V, O_V - O_V) = -Qi.inverse().toRotationMatrix();
            jacobian_speedbias_i.block<3, 3>(O_V, O_BA - O_V) = -dv_dba;
            jacobian_speedbias_i.block<3, 3>(O_V, O_BG - O_V) = -dv_dbg;

            jacobian_speedbias_i.block<3, 3>(O_BA, O_BA - O_V) = -Eigen::Matrix3d::Identity();

            jacobian_speedbias_i.block<3, 3>(O_BG, O_BG - O_V) = -Eigen::Matrix3d::Identity();

            jacobian_speedbias_i = sqrt_info_matrix * jacobian_speedbias_i;

            ROS_ASSERT(fabs(jacobian_speedbias_i.maxCoeff()) < 1e8);
            ROS_ASSERT(fabs(jacobian_speedbias_i.minCoeff()) < 1e8);
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[2]);
            jacobian_pose_j.setZero();

            jacobian_pose_j.block<3, 3>(O_P, O_P) = Qi.inverse().toRotationMatrix();

#if 0
                jacobian_pose_j.block<3, 3>(O_R, O_R) = Eigen::Matrix3d::Identity();
#else
            Eigen::Quaterniond corrected_delta_q =
                imu_preintegration_->delta_q_ * Utility::deltaQ(dq_dbg * (Bgi - imu_preintegration_->linearized_bg_));
            jacobian_pose_j.block<3, 3>(O_R, O_R) = Utility::Qleft(
                                                        corrected_delta_q.inverse() * Qi.inverse() * Qj)
                                                        .bottomRightCorner<3, 3>();
#endif

            jacobian_pose_j = sqrt_info_matrix * jacobian_pose_j;

            ROS_ASSERT(fabs(jacobian_pose_j.maxCoeff()) < 1e8);
            ROS_ASSERT(fabs(jacobian_pose_j.minCoeff()) < 1e8);
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Matrix<double, 15, 9, Eigen::RowMajor>> jacobian_speedbias_j(jacobians[3]);
            jacobian_speedbias_j.setZero();

            jacobian_speedbias_j.block<3, 3>(O_V, O_V - O_V) = Qi.inverse().toRotationMatrix();

            jacobian_speedbias_j.block<3, 3>(O_BA, O_BA - O_V) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j.block<3, 3>(O_BG, O_BG - O_V) = Eigen::Matrix3d::Identity();

            jacobian_speedbias_j = sqrt_info_matrix * jacobian_speedbias_j;

            ROS_ASSERT(fabs(jacobian_speedbias_j.maxCoeff()) < 1e8);
            ROS_ASSERT(fabs(jacobian_speedbias_j.minCoeff()) < 1e8);
        }
    }
    return true;
}
} // namespace gp_lio