//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_GLOBALMAPPER_H
#define SRC_GLOBALMAPPER_H

#include "gp_lio/factor/GpPriorFactor.h"
#include "gp_lio/feature/LidarHandcraftFeature.h"
#include "gp_lio/FeatureVector.h"
#include "gp_lio/mapper/KeyFrame.h"
#include "gp_lio/estimator/Parameters.h"
#include "gp_lio/utility/Utility.h"
#include "gp_lio/factor/PoseLocalParameterization.h"
#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/mapper/LocalMapMatcher.h"
#include <gp_lio/utility/file_manager.h>
#include "LocalCartesian.hpp"
#include "gp_lio/mapper/globalOpt.h"

#include <assert.h>
#include <stdio.h>
#include <thread>
#include <mutex>
#include <unistd.h>





// pcl
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "pcl/registration/ndt.h"

// ros
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/NavSatFix.h>

//cv
#include <opencv2/opencv.hpp>

// ceres
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>
#include <ceres/types.h>

#include <utility>

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

    class GlobalMapper {

    public:

        GlobalMapper();

        ~GlobalMapper();

        void LoadConfig(ros::NodeHandle &nh);

        void GnssMsgsCallback(const sensor_msgs::NavSatFixPtr& gnss_msg);

        void DescriptorCallback(const gp_lio::FeatureVector& descriptor_msg);

        void Run();

        bool AddNewKF(const KeyFrame& newKeyFrame);

        State GetUpdatePose();

        void GetClosedMap(PointCloudT& closed_edge_map,
                          PointCloudT& closed_surf_map,
                          PointCloudT& closed_ground_map,
                          std::vector<PlaneFeature>& closed_planes,
                          State state);

        void LoadPosegraph();

        void SavePosegraph();


        bool IsLoopClosed();

        bool GetRelocalizationInfo(std::vector<std::pair<bool,Frame>>& loopframe_v);

        bool FastRelocalization(LoopClosurePair & loopcp);

        void FastRelocalization(State & updated_state);




    private:

        void SearchTrajecoryOffsets();

        void PubCloudForDescriptor();

        void LoopClosureDetection();

        void UpdateDistanceMat();

        void RecoveryDistanceMat();

        void SearchScore(std::vector<int> query_traj,
                         std::vector<int> smallest_ref_idx_seg,
                         std::vector<std::vector<int>> ref_trajs,
                         float& score, std::vector<int>& chosed_ref_traj);

        void WindowThresholding(std::vector<float> score_v, float& uniqueness, int& traj_idx);

        double ComputeDistance(const std::vector<float>& v_1, const std::vector<float>& v_2);

        bool RelativeTransformation(LoopClosurePair& new_pair);

        bool RelativeTransformationNDT(LoopClosurePair& new_pair);

        void GroundRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud);

        void PlaneRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud);

        void AllCloudRegistration(LoopClosurePair& new_pair, PointCloudT ref_cloud, PointCloudT query_cloud);

        RegistrationResult ICPRegistration( PointCloudT ref_cloud, PointCloudT query_cloud, Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity());

        PointCloudT ChangeCloud(pcl::PointCloud<pcl::PointXYZ> cloud);

        void ShowClouds(PointCloudT ref_cloud, PointCloudT query_cloud, std::string type);

        void Optimization();

        void Optimize4DOF();

        void UpdateMap();

        void Publish();

        void PublishReusedMap();

        void MapMerge();

        void PubGlobalPath();







    private:

        LocalMapMatcher local_map_matcher_;
        bool loop_closed_;
        bool add_new_kf_;
        bool to_detect_;
        int last_kf_idx_ = 0;
        size_t idx_;

        ros::NodeHandle nh_;

        std::vector<KeyFrame> global_KFs_v_;
        std::vector<PlaneFeature> global_planes_v;

        ros::Subscriber sub_kf_gnss_;
        ros::Subscriber sub_kf_descriptor_;
        ros::Publisher pub_kf_cloud_;
        ros::Publisher pub_pose_graph_old_;
        ros::Publisher pub_pose_graph_new_;
        ros::Publisher pub_global_path_;

        std::vector<LoopClosurePair> loop_pairs_v_;
        int loop_count_;

        cv::Mat distance_Mat_;
        std::vector<int> smallest_ref_idx_;
        std::vector<int> query_base_trajs_;
        std::vector<std::vector<int>> ref_base_trajs_;
        pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_;
        pcl::VoxelGrid<pcl::PointXYZI> map_downsize_filter_;

        ros::Publisher all_feature_cloud_pub_;
        ros::Publisher all_ground_cloud_pub_;
        ros::Publisher all_plane_cloud_pub_;
        ros::Publisher loop_pub_;
        ros::Publisher all_cloud_pub_;

        PointCloudT::Ptr all_feature_cloud_map_;
        PointCloudT::Ptr all_ground_cloud_map_;
        PointCloudT::Ptr all_plane_cloud_map_;
        PointCloudT::Ptr all_cloud_map_;

        bool add_latest_map_;

        int last_optimized_idx_;
        FileManager file_manager_;
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeHistoryKeyPoses;
        // reuse map
        int global_index_;
        bool optimize_pose_graph_;
        bool to_merge_map;
        bool map_merged;
        std::pair<int,State> merge_point;
        bool build_tree_;
        int last_closed_index;
        std::queue<sensor_msgs::NavSatFixPtr> gnss_buf;
        double kf_timestamp;
        std::mutex gnss_mutex;
        pcl::NormalDistributionsTransform<pcl::PointXYZI,pcl::PointXYZI>::Ptr ndt_ptr_;
        // global optimization
        GlobalOptimization global_estimator_;





    public:
        bool initialized_;
        bool need_update_;
        ExternalParameters extrinsic_parameters_;



    };
}



#endif //SRC_GLOBALMAPPER_H
