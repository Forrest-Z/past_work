//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/factor/LidarFactor.h"

namespace gp_lio{

    SurfFactor::SurfFactor(const gp_lio::FeatureCorPairs &featureCorPairs, const double &weight) {

        plane_normal_ = featureCorPairs.dir_vec_;
        surf_feature_ = Eigen::Vector3d(featureCorPairs.feature_.x,
                                        featureCorPairs.feature_.y,
                                        featureCorPairs.feature_.z);
        plane_center_ = Eigen::Vector3d(featureCorPairs.corresponding_center_.x,
                                        featureCorPairs.corresponding_center_.y,
                                        featureCorPairs.corresponding_center_.z);
        distance_ = featureCorPairs.distance_;
        weight_ = weight;

    }

    bool SurfFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {

        Eigen::Vector3d P0(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Q0(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d P1(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Q1(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d BL_P(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond BL_Q(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

        Eigen::Vector3d LB_P = - BL_Q.toRotationMatrix().transpose() * BL_P;
        Eigen::Quaterniond  LB_Q = BL_Q.conjugate();

        Eigen::Quaterniond QlP1vot = Q0 * LB_Q.conjugate();
        Eigen::Vector3d PlP1vot = P0 - QlP1vot * LB_P;

        Eigen::Quaterniond Qli = Q1 * LB_Q.conjugate();
        Eigen::Vector3d Pli = P1 - Qli * LB_P;

        Eigen::Quaterniond QlP1 = QlP1vot.conjugate() * Qli;
        Eigen::Vector3d PlP1 = QlP1vot.conjugate() * (Pli - PlP1vot);

        double residual = plane_normal_.transpose() * (QlP1 * surf_feature_ + PlP1 ) + distance_;

        residuals[0] = weight_ * residual;

        if (jacobians){

            Eigen::Matrix3d R0 = Q0.toRotationMatrix();
            Eigen::Matrix3d R1 = Q1.toRotationMatrix();
            Eigen::Matrix3d LB_R = LB_Q.toRotationMatrix();

            if (jacobians[0]){

                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_0(jacobians[0]);
                Eigen::Matrix<double, 1, 6> jaco_0;

                jaco_0.leftCols<3>() = - plane_normal_.transpose() * LB_R * R0.transpose();
                jaco_0.rightCols<3>() = plane_normal_.transpose() * LB_R * (Utility::SkewSymmetric(R0.transpose() * R1 * LB_R.transpose() * (surf_feature_ - LB_P))
                                                                + Utility::SkewSymmetric(R0.transpose() * (P1 - P0)));

                jacobian_pose_0.setZero();
                jacobian_pose_0.leftCols<6>() = weight_ * jaco_0;
                jacobian_pose_0.rightCols<1>().setZero();
            }

            if (jacobians[1]){

            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_1(jacobians[1]);
            Eigen::Matrix<double, 1, 6> jaco_1;

            jaco_1.leftCols<3>() = plane_normal_.transpose() * LB_R * R0.transpose();
            jaco_1.rightCols<3>() = plane_normal_.transpose() * LB_R * R0.transpose() * R1 * (-Utility::SkewSymmetric(LB_R.transpose() * surf_feature_)
                                                                                                + Utility::SkewSymmetric(LB_R.transpose() * LB_P));

            jacobian_pose_1.setZero();
            jacobian_pose_1.leftCols<6>() = weight_ * jaco_1;
            jacobian_pose_1.rightCols<1>().setZero();
            }

            if (jacobians[2]){

                Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_ex(jacobians[2]);
                Eigen::Matrix<double, 1, 6> jaco_ex;
                Eigen::Matrix3d I3x3;
                I3x3.setIdentity();

                jaco_ex.leftCols<3>() = plane_normal_.transpose() * (I3x3 - LB_R * R0.transpose() * R1 * LB_R.transpose());
                jaco_ex.rightCols<3>() = plane_normal_.transpose() * LB_R * (-Utility::SkewSymmetric(R0.transpose() * R1 * LB_R.transpose() * (surf_feature_ - LB_P))
                                                                               + R0.transpose() * R1 * Utility::SkewSymmetric(LB_R.transpose() * (surf_feature_ - LB_P))
                                                                               - Utility::SkewSymmetric(R0.transpose() * (P1 - P0)));

                jacobian_pose_ex.setZero();
                jacobian_pose_ex.leftCols<6>() = weight_ * jaco_ex;
                jacobian_pose_ex.rightCols<1>().setZero();
            }

        }
        return true;
    }
}