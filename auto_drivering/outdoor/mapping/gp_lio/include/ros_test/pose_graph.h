//
// Created by wchen on 2020/7/21.
//

#ifndef GP_LIO_POSE_GRAPH_H
#define GP_LIO_POSE_GRAPH_H

#include "gp_lio/factor/PoseLocalParameterization.h"
#include "gp_lio/utility/Utility.h"
#include "gp_lio/utility/tic_toc.h"

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>

// ceres
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>

namespace gp_lio{
    class PoseGraph3dErrorTerm {
    public:
        PoseGraph3dErrorTerm(Eigen::Vector3d rel_p, Eigen::Quaterniond rel_q,
                             Eigen::Matrix<double, 6, 6>  sqrt_information)
                : rel_p_(rel_p), rel_qua_(rel_q), sqrt_information_(std::move(sqrt_information)) {

        }

        template <typename T>
        bool operator()(const T* const pose_A, const T* const pose_B,
                        T* residuals_ptr) const {

            const T pose_A_p[3] = {pose_A[0], pose_A[1], pose_A[2]};
            const T pose_A_q[4] = {pose_A[3], pose_A[4], pose_A[5], pose_A[6]};

            const T pose_B_p[3] = {pose_B[0], pose_B[1], pose_B[2]};
            const T pose_B_q[4] = {pose_B[3], pose_B[4], pose_B[5], pose_B[6]};

            Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_A(pose_A_p);
            Eigen::Map<const Eigen::Quaternion<T> > q_A(pose_A_q);

            Eigen::Map<const Eigen::Matrix<T, 3, 1> > p_B(pose_B_p);
            Eigen::Map<const Eigen::Quaternion<T> > q_B(pose_B_q);

            // Compute the relative transformation between the two frames.
            Eigen::Quaternion<T> q_A_inverse = q_A.conjugate();
            Eigen::Quaternion<T> q_AB_estimated = q_A_inverse * q_B;

            // Represent the displacement between the two frames in the A frame.
            Eigen::Matrix<T, 3, 1> p_AB_estimated = q_A_inverse * (p_B - p_A);

            // Compute the error between the two orientation estimates.
            Eigen::Quaternion<T> delta_q =
                    rel_qua_.template cast<T>() * q_AB_estimated.conjugate();

            // Compute the residuals.
            // [ position         ]   [ delta_p          ]
            // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
            Eigen::Map<Eigen::Matrix<T, 6, 1> > residuals(residuals_ptr);
            residuals.template block<3, 1>(0, 0) =
                    p_AB_estimated - rel_p_.template cast<T>();
            residuals.template block<3, 1>(3, 0) = T(2.0) * delta_q.vec();

            // Scale the residuals by the measurement uncertainty.
            residuals.applyOnTheLeft(sqrt_information_.template cast<T>());

            return true;
        }

        static ceres::CostFunction* Create(
                const Eigen::Vector3d rel_p, Eigen::Quaterniond rel_q,
                const Eigen::Matrix<double, 6, 6>& sqrt_information) {
            return new ceres::AutoDiffCostFunction<PoseGraph3dErrorTerm, 6, 7, 7>(
                    new PoseGraph3dErrorTerm(rel_p, rel_q, sqrt_information));
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    private:
        // The measurement for the position of B relative to A in the A frame.
        Eigen::Quaterniond rel_qua_;
        Eigen::Vector3d rel_p_;
        // The square root of the measurement information matrix.
        const Eigen::Matrix<double, 6, 6> sqrt_information_;
    };

// from VINS-MONO
    template <typename T>
    T NormalizeAngle(const T& angle_degrees) {
        if (angle_degrees > T(180.0))
            return angle_degrees - T(360.0);
        else if (angle_degrees < T(-180.0))
            return angle_degrees + T(360.0);
        else
            return angle_degrees;
    };

    class AngleLocalParameterization {
    public:

        template <typename T>
        bool operator()(const T* theta_radians, const T* delta_theta_radians,
                        T* theta_radians_plus_delta) const {
            *theta_radians_plus_delta =
                    NormalizeAngle(*theta_radians + *delta_theta_radians);

            return true;
        }

        static ceres::LocalParameterization* Create() {
            return (new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                    1, 1>);
        }
    };

    template <typename T>
    void YawPitchRollToRotationMatrix(const T yaw, const T pitch, const T roll, T R[9])
    {

        T y = yaw / T(180.0) * T(M_PI);
        T p = pitch / T(180.0) * T(M_PI);
        T r = roll / T(180.0) * T(M_PI);


        R[0] = cos(y) * cos(p);
        R[1] = -sin(y) * cos(r) + cos(y) * sin(p) * sin(r);
        R[2] = sin(y) * sin(r) + cos(y) * sin(p) * cos(r);
        R[3] = sin(y) * cos(p);
        R[4] = cos(y) * cos(r) + sin(y) * sin(p) * sin(r);
        R[5] = -cos(y) * sin(r) + sin(y) * sin(p) * cos(r);
        R[6] = -sin(p);
        R[7] = cos(p) * sin(r);
        R[8] = cos(p) * cos(r);
    };

    template <typename T>
    void RotationMatrixTranspose(const T R[9], T inv_R[9])
    {
        inv_R[0] = R[0];
        inv_R[1] = R[3];
        inv_R[2] = R[6];
        inv_R[3] = R[1];
        inv_R[4] = R[4];
        inv_R[5] = R[7];
        inv_R[6] = R[2];
        inv_R[7] = R[5];
        inv_R[8] = R[8];
    };

    template <typename T>
    void RotationMatrixRotatePoint(const T R[9], const T t[3], T r_t[3])
    {
        r_t[0] = R[0] * t[0] + R[1] * t[1] + R[2] * t[2];
        r_t[1] = R[3] * t[0] + R[4] * t[1] + R[5] * t[2];
        r_t[2] = R[6] * t[0] + R[7] * t[1] + R[8] * t[2];
    };

    struct FourDOFError
    {
        FourDOFError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){}

        template <typename T>
        bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
        {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x));
            residuals[1] = (t_i_ij[1] - T(t_y));
            residuals[2] = (t_i_ij[2] - T(t_z));
            residuals[3] = NormalizeAngle(yaw_j[0] - yaw_i[0] - T(relative_yaw));

            return true;
        }

        static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                           const double relative_yaw, const double pitch_i, const double roll_i)
        {
            return (new ceres::AutoDiffCostFunction<
                    FourDOFError, 4, 1, 3, 1, 3>(
                    new FourDOFError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;

    };

    struct FourDOFWeightError
    {
        FourDOFWeightError(double t_x, double t_y, double t_z, double relative_yaw, double pitch_i, double roll_i)
                :t_x(t_x), t_y(t_y), t_z(t_z), relative_yaw(relative_yaw), pitch_i(pitch_i), roll_i(roll_i){
            weight = 1;
        }

        template <typename T>
        bool operator()(const T* const yaw_i, const T* ti, const T* yaw_j, const T* tj, T* residuals) const
        {
            T t_w_ij[3];
            t_w_ij[0] = tj[0] - ti[0];
            t_w_ij[1] = tj[1] - ti[1];
            t_w_ij[2] = tj[2] - ti[2];

            // euler to rotation
            T w_R_i[9];
            YawPitchRollToRotationMatrix(yaw_i[0], T(pitch_i), T(roll_i), w_R_i);
            // rotation transpose
            T i_R_w[9];
            RotationMatrixTranspose(w_R_i, i_R_w);
            // rotation matrix rotate point
            T t_i_ij[3];
            RotationMatrixRotatePoint(i_R_w, t_w_ij, t_i_ij);

            residuals[0] = (t_i_ij[0] - T(t_x)) * T(weight);
            residuals[1] = (t_i_ij[1] - T(t_y)) * T(weight);
            residuals[2] = (t_i_ij[2] - T(t_z)) * T(weight);
            residuals[3] = NormalizeAngle((yaw_j[0] - yaw_i[0] - T(relative_yaw))) * T(weight) / T(10.0);

            return true;
        }

        static ceres::CostFunction* Create(const double t_x, const double t_y, const double t_z,
                                           const double relative_yaw, const double pitch_i, const double roll_i)
        {
            return (new ceres::AutoDiffCostFunction<
                    FourDOFWeightError, 4, 1, 3, 1, 3>(
                    new FourDOFWeightError(t_x, t_y, t_z, relative_yaw, pitch_i, roll_i)));
        }

        double t_x, t_y, t_z;
        double relative_yaw, pitch_i, roll_i;
        double weight;

    };

    struct Pose{
        Eigen::Quaterniond quaternion_;
        Eigen::Vector3d position_;
        Eigen::Matrix4d Get_T(){
            Eigen::Matrix4d mat4d;
            mat4d.setIdentity();
            mat4d.block<3, 3>(0, 0) = quaternion_.toRotationMatrix();
            mat4d.block<3, 1>(0, 3) = position_;
            return mat4d;
        };
    };
    struct loop_pair{
        std::pair<int, int> pair_idx_;
        Pose rel_pose;
    };

    class pose_graph {
    public:
        pose_graph();

        ~pose_graph();

        void LoadGraph();

        void Run();

    private:

        void Optimization();

        void Optimize4DOF();

        void Publish();


    private:

        ros::NodeHandle nh_;

        ros::Publisher pub_pose_graph_;
        ros::Publisher pub_origin_pose_graph_;

        std::vector<Pose> graph_pose_v_;
        std::vector<loop_pair> loops_v_;

        PointCloudT origin_pose_graph_;

        bool optimized_;

    };

}




#endif //GP_LIO_POSE_GRAPH_H
