//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_LIDARFACTOR_H
#define SRC_LIDARFACTOR_H

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "gp_lio/utility/Utility.h"

namespace gp_lio {

    struct EdgeFeatureCostFunctor {

    public:
        EdgeFeatureCostFunctor() = delete;

        EdgeFeatureCostFunctor(const FeatureCorPairs &featureCorPairs, const double &weight) {

            weight_ = weight;
            line_dir_ = featureCorPairs.dir_vec_;
            edge_feature_ = Eigen::Vector3d(featureCorPairs.feature_.x,
                                            featureCorPairs.feature_.y,
                                            featureCorPairs.feature_.z);
            line_center_ = Eigen::Vector3d(featureCorPairs.corresponding_center_.x,
                                           featureCorPairs.corresponding_center_.y,
                                           featureCorPairs.corresponding_center_.z);
        }

        template<typename T>
        bool operator()(const T *const pose_i, const T *const pose_ex, T *residuals) const {

            const T Pi[3] = {pose_i[0], pose_i[1], pose_i[2]};
            const T Qi[4] = {pose_i[3], pose_i[4], pose_i[5], pose_i[6]}; // in ceres, quaternion order is (x,y,z,w)

            const T BL_P[3] = {pose_ex[0], pose_ex[1], pose_ex[2]};
            const T BL_Q[4] = {pose_ex[3], pose_ex[4], pose_ex[5], pose_ex[6]};

            const T feature[3] = {T(0) + edge_feature_.x(), T(0) + edge_feature_.y(), T(0) + edge_feature_.z()};

            //            double residual = (line_dir_.cross(Qi * BL_Q * edge_feature_ + Qi * BL_P + Pi - line_center_)).norm();

            T newFeature_1[3];
            ceres::QuaternionRotatePoint(BL_Q, feature, newFeature_1);

            T newFeature_2[3];
            ceres::QuaternionRotatePoint(Qi, newFeature_1, newFeature_2);
            T newPosi_1[3];
            ceres::QuaternionRotatePoint(Qi, BL_P, newPosi_1);
            T newPosi_2[3] = {Pi[0] - line_center_.x(), Pi[1] - line_center_.y(), Pi[2] - line_center_.z()};

            T newPosi_3[3] = {newFeature_2[0] + newPosi_1[0] + newPosi_2[0],
                              newFeature_2[1] + newPosi_1[1] + newPosi_2[1],
                              newFeature_2[2] + newPosi_1[2] + newPosi_2[2]};

            T x[3] = {T(0) + line_dir_.x(), T(0) + line_dir_.y(), T(0) + line_dir_.z()};

            T x_cross_y[3];
            ceres::CrossProduct(x, newPosi_3, x_cross_y);

            residuals[0] = weight_ * ceres::sqrt(
                    x_cross_y[0] * x_cross_y[0] + x_cross_y[1] * x_cross_y[1] + x_cross_y[2] * x_cross_y[2]);

            return true;
        }

    private:
        Eigen::Vector3d line_dir_;
        Eigen::Vector3d edge_feature_;
        Eigen::Vector3d line_center_;
        double weight_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

// 1 - residual; 7 - imu pose 0; 7 - imu pose 1; 7 - extrinsic tranformation
    class SurfFactor : public ceres::SizedCostFunction<1, 7, 7, 7> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SurfFactor() = delete;

        SurfFactor(const FeatureCorPairs &featureCorPairs, const double &weight);

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;

    private:
        Eigen::Vector3d plane_normal_;
        Eigen::Vector3d surf_feature_;
        Eigen::Vector3d plane_center_;
        double distance_;
        double weight_;
    };

// New Factor design
    struct FeatureCostFunctorNew {

    public:
        FeatureCostFunctorNew() = delete;

        FeatureCostFunctorNew(const FeatureCorPairs &featureCorPairs, const double &weight) {

            weight_ = weight;
            featureCorPairs_ = featureCorPairs;
        }

        template<typename T>
        bool operator()(const T *const pose_0, const T *const pose_1, const T *const pose_ex, T *residuals) const {

            const T P0[3] = {pose_0[0], pose_0[1], pose_0[2]};
            const T Q0[4] = {pose_0[3], pose_0[4], pose_0[5], pose_0[6]};// in ceres, quaternion order is (x,y,z,w)

            const T P1[3] = {pose_1[0], pose_1[1], pose_1[2]};
            const T Q1[4] = {pose_1[3], pose_1[4], pose_1[5], pose_1[6]};

            const T BL_P[3] = {pose_ex[0], pose_ex[1], pose_ex[2]};
            const T BL_Q[4] = {pose_ex[3], pose_ex[4], pose_ex[5], pose_ex[6]};

            const T feature[3] = {T(0) + double(featureCorPairs_.feature_.x),
                                  T(0) + double(featureCorPairs_.feature_.y),
                                  T(0) + double(featureCorPairs_.feature_.z)};

            // transform feature from {L_1} to {L_0}
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_0(P0);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_1(P1);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bl_p(BL_P);

            Eigen::Map<const Eigen::Quaternion<T>> q_0(Q0);
            Eigen::Map<const Eigen::Quaternion<T>> q_1(Q1);
            Eigen::Map<const Eigen::Quaternion<T>> bl_q(BL_Q);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> fea(feature);

            Eigen::Quaternion<T> q_inverse = (q_0 * bl_q).conjugate();

            Eigen::Quaternion<T> rel_q = q_inverse * (q_1 * bl_q);
            Eigen::Matrix<T, 3, 1> rel_p = q_inverse * (q_1 * bl_p + p_1) -
                                           q_inverse * (q_0 * bl_p + p_0);

            Eigen::Matrix<T, 3, 1> X_0 = rel_q * fea + rel_p;

            if (featureCorPairs_.feature_type_ == FeatureType::Edge) {

                const T cor_1[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(0).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).z)};

                const T cor_2[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(1).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).z)};
                // residual from LOAM
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_1(cor_1);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_2(cor_2);

                Eigen::Matrix<T, 3, 1> numerator = ((X_0 - X_1).cross((X_0 - X_2)));
                Eigen::Matrix<T, 3, 1> denominator = (X_1 - X_2);

                residuals[0] = T(weight_) * numerator.x() / denominator.norm();
                residuals[1] = T(weight_) * numerator.y() / denominator.norm();
                residuals[2] = T(weight_) * numerator.z() / denominator.norm();

            } else {

                const T cor_1[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(0).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).z)};

                const T cor_2[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(1).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).z)};

                const T cor_3[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(2).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(2).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(2).z)};

                // residual from LOAM
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_1(cor_1);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_2(cor_2);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_3(cor_3);

                Eigen::Matrix<T, 3, 1> plane_norm = ((X_1 - X_2).cross(X_1 - X_3));

                residuals[0] = T(weight_) * (X_0 - X_1).dot(plane_norm/plane_norm.norm());
            }

            return true;
        }

    private:

        FeatureCorPairs featureCorPairs_;
        double weight_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct FeatureCostFunctorMap {

    public:
        FeatureCostFunctorMap() = delete;

        FeatureCostFunctorMap(const FeatureCorPairs &featureCorPairs, const double &weight) {

            weight_ = weight;
            featureCorPairs_ = featureCorPairs;
        }

        template<typename T>
        bool operator()(const T *const pose_1, const T *const pose_ex, T *residuals) const {

            const T P1[3] = {pose_1[0], pose_1[1], pose_1[2]};
            const T Q1[4] = {pose_1[3], pose_1[4], pose_1[5], pose_1[6]};

            const T BL_P[3] = {pose_ex[0], pose_ex[1], pose_ex[2]};
            const T BL_Q[4] = {pose_ex[3], pose_ex[4], pose_ex[5], pose_ex[6]};

            const T feature[3] = {T(0) + double(featureCorPairs_.feature_.x),
                                  T(0) + double(featureCorPairs_.feature_.y),
                                  T(0) + double(featureCorPairs_.feature_.z)};

            // transform feature from {L} to {W}
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_1(P1);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bl_p(BL_P);

            Eigen::Map<const Eigen::Quaternion<T>> q_1(Q1);
            Eigen::Map<const Eigen::Quaternion<T>> bl_q(BL_Q);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> fea(feature);

            Eigen::Quaternion<T> rel_q = q_1 * bl_q;
            Eigen::Matrix<T, 3, 1> rel_p = q_1 * bl_p + p_1;

            Eigen::Matrix<T, 3, 1> X_0 = rel_q * fea + rel_p;

            if (featureCorPairs_.feature_type_ == FeatureType::Edge) {

                const T cor_1[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(0).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).z)};

                const T cor_2[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(1).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).z)};
                // residual from LOAM
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_1(cor_1);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_2(cor_2);

                Eigen::Matrix<T, 3, 1> numerator = ((X_0 - X_1).cross((X_0 - X_2)));
                Eigen::Matrix<T, 3, 1> denominator = (X_1 - X_2);

                residuals[0] = T(weight_) * numerator.x() / denominator.norm();
                residuals[1] = T(weight_) * numerator.y() / denominator.norm();
                residuals[2] = T(weight_) * numerator.z() / denominator.norm();

            } else {

                const T cor_1[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(0).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(0).z)};

                const T cor_2[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(1).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(1).z)};

                const T cor_3[3] = {T(0) + double(featureCorPairs_.corresponding_points_.at(2).x),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(2).y),
                                    T(0) + double(featureCorPairs_.corresponding_points_.at(2).z)};

                // residual from LOAM
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_1(cor_1);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_2(cor_2);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> X_3(cor_3);

                Eigen::Matrix<T, 3, 1> plane_norm = ((X_1 - X_2).cross(X_1 - X_3));

                residuals[0] = T(weight_) * (X_0 - X_1).dot(plane_norm/plane_norm.norm());
            }

            return true;
        }

    private:

        FeatureCorPairs featureCorPairs_;
        double weight_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct PlaneCostFunctor {

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PlaneCostFunctor() = delete;

        PlaneCostFunctor(Eigen::Vector3f &cp_0, Eigen::Vector3f &cp_1, double weight) {
            cp_0_ = cp_0.cast<double>();
            cp_1_ = cp_1.cast<double>();
            weight_ = weight;
        }

        template<typename T>
        bool operator()(const T *const pose_0, const T *const pose_1, const T *const pose_ex, T *residuals) const {

            const T P0[3] = {pose_0[0], pose_0[1], pose_0[2]};
            const T Q0[4] = {pose_0[3], pose_0[4], pose_0[5], pose_0[6]};// in ceres, quaternion order is (x,y,z,w)

            const T P1[3] = {pose_1[0], pose_1[1], pose_1[2]};
            const T Q1[4] = {pose_1[3], pose_1[4], pose_1[5], pose_1[6]};

            const T BL_P[3] = {pose_ex[0], pose_ex[1], pose_ex[2]};
            const T BL_Q[4] = {pose_ex[3], pose_ex[4], pose_ex[5], pose_ex[6]};


            const T cp_0[3] = {T(0) + double(cp_0_.x()),
                               T(0) + double(cp_0_.y()),
                               T(0) + double(cp_0_.z())};

            const T cp_1[3] = {T(0) + double(cp_1_.x()),
                               T(0) + double(cp_1_.y()),
                               T(0) + double(cp_1_.z())};

            // transform feature from {L_1} to {L_0}
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_0(P0);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_1(P1);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bl_p(BL_P);

            Eigen::Map<const Eigen::Quaternion<T>> q_0(Q0);
            Eigen::Map<const Eigen::Quaternion<T>> q_1(Q1);
            Eigen::Map<const Eigen::Quaternion<T>> bl_q(BL_Q);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> cp_0_v(cp_0);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> cp_1_v(cp_1);

            Eigen::Quaternion<T> q_inverse = (q_0 * bl_q).conjugate();

            Eigen::Quaternion<T> rel_q = q_inverse * (q_1 * bl_q);
            Eigen::Matrix<T, 3, 1> rel_p = q_inverse * (q_1 * bl_p + p_1) -
                                           q_inverse * (q_0 * bl_p + p_0);

            Eigen::Matrix<T, 3, 1> rel_p_inv = -rel_q.toRotationMatrix().transpose() * rel_p;

            Eigen::Matrix<T, 3, 1> cp_1_v_normlized = cp_1_v / cp_1_v.norm();

            Eigen::Matrix<T, 3, 1> n_new = rel_q.toRotationMatrix() * cp_1_v_normlized;

            T d_new =  cp_1_v.norm() + rel_p_inv[0] * cp_1_v_normlized[0] + rel_p_inv[1] * cp_1_v_normlized[1] + rel_p_inv[2] * cp_1_v_normlized[2];

            Eigen::Matrix<T, 3, 1> residual_v = cp_0_v - n_new * d_new;

            residuals[0] = residual_v(0) * T(weight_);
            residuals[1] = residual_v(1) * T(weight_);
            residuals[2] = residual_v(2) * T(weight_);

//            std::cout << " residual: [" << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << "] " << std::endl;
            return true;
        }

    private:

        Eigen::Vector3d cp_0_;
        Eigen::Vector3d cp_1_;
        double weight_;

    };

    struct PlaneCostFunctorMap {

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PlaneCostFunctorMap() = delete;

        PlaneCostFunctorMap(Eigen::Vector3f &cp_0, Eigen::Vector3f &cp_1, double weight) {
            cp_0_ = cp_0.cast<double>();
            cp_1_ = cp_1.cast<double>();
            weight_ = weight;
        }

        template<typename T>
        bool operator()(const T *const pose_1, const T *const pose_ex, T *residuals) const {

            const T P1[3] = {pose_1[0], pose_1[1], pose_1[2]};
            const T Q1[4] = {pose_1[3], pose_1[4], pose_1[5], pose_1[6]};

            const T BL_P[3] = {pose_ex[0], pose_ex[1], pose_ex[2]};
            const T BL_Q[4] = {pose_ex[3], pose_ex[4], pose_ex[5], pose_ex[6]};


            const T cp_0[3] = {T(0) + double(cp_0_.x()),
                               T(0) + double(cp_0_.y()),
                               T(0) + double(cp_0_.z())};

            const T cp_1[3] = {T(0) + double(cp_1_.x()),
                               T(0) + double(cp_1_.y()),
                               T(0) + double(cp_1_.z())};

            // transform feature from {L} to {W}
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> p_1(P1);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> bl_p(BL_P);

            Eigen::Map<const Eigen::Quaternion<T>> q_1(Q1);
            Eigen::Map<const Eigen::Quaternion<T>> bl_q(BL_Q);

            Eigen::Map<const Eigen::Matrix<T, 3, 1>> cp_0_v(cp_0);
            Eigen::Map<const Eigen::Matrix<T, 3, 1>> cp_1_v(cp_1);

            Eigen::Quaternion<T> rel_q = q_1 * bl_q;
            Eigen::Matrix<T, 3, 1> rel_p = q_1 * bl_p + p_1;

            Eigen::Matrix<T, 3, 1> rel_p_inv = -rel_q.toRotationMatrix().transpose() * rel_p;

            Eigen::Matrix<T, 3, 1> cp_1_v_normlized = cp_1_v / cp_1_v.norm();

            Eigen::Matrix<T, 3, 1> n_new = rel_q.toRotationMatrix() * cp_1_v_normlized;

            T d_new =  cp_1_v.norm() + rel_p_inv[0] * cp_1_v_normlized[0] + rel_p_inv[1] * cp_1_v_normlized[1] + rel_p_inv[2] * cp_1_v_normlized[2];

            Eigen::Matrix<T, 3, 1> residual_v = cp_0_v - n_new * d_new;

            residuals[0] = residual_v(0) * T(weight_);
            residuals[1] = residual_v(1) * T(weight_);
            residuals[2] = residual_v(2) * T(weight_);

//            std::cout << " residual: [" << residuals[0] << ", " << residuals[1] << ", " << residuals[2] << "] " << std::endl;
            return true;
        }

    private:

        Eigen::Vector3d cp_0_;
        Eigen::Vector3d cp_1_;
        double weight_;

    };

}// namespace gp_lio
#endif //SRC_LIDARFACTOR_H
