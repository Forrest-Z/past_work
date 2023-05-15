//
// Created by wchen on 2020/8/11.
//

#ifndef GP_LIO_LOCALMAPMATCHER_H
#define GP_LIO_LOCALMAPMATCHER_H

#include <iostream>
#include <time.h>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdlib>
#include <chrono>


#include "gp_lio/utility/Utility.h"

#include <pcl/common/common.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/pcl_search.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>

namespace gp_lio{

    typedef pcl::PointXYZ PointTX; // use PointXYZ is more time-efficient
    typedef pcl::PointCloud<PointTX> PointCloudTX;

    struct corrTab{ //correspondence table, including the indices in corrIdx and the upperbound
        int idxS; //first idx in corrIdx
        int idxT; //second idx in corrIdx
        int upBnd; //upperbound for this correspondences
        int lwBnd;
        double disFeature; //distance between the feature descriptors
    };

    struct line_interval{
        double t_min;
        double t_max;
        int cor_idx;
        bool empty;
        line_interval():t_min(0), t_max(0), empty(true), cor_idx(-1){};
        line_interval(double left, double right):t_min(left), t_max(right), cor_idx(-1), empty(false){};

        line_interval &operator = (const line_interval &other){
            t_min = other.t_min;
            t_max = other.t_max;
            cor_idx = other.cor_idx;
            empty = other.empty;
            return *this;
        }

    };

    struct intervalEnd{
        double location; //location of the end point
        bool isStart; //is this end point a starting point of an interval
        int corrIdx;
        void formIntervalEnd(const double &location_in, const bool &isStart_in, const int &corrIdx_in){
            location = location_in;
            isStart = isStart_in;
            corrIdx = corrIdx_in;
        }
    };

    struct MaxStabbingResult{

        int inlier_count;
        int target_plane_idx;
        int source_plane_idx;
        double tx;
        Eigen::Matrix4d T_target;
        Eigen::Matrix4d T_source;
        std::vector<int> inlier_corr_idxs;

        MaxStabbingResult(): inlier_count(-1), target_plane_idx(0), source_plane_idx(0), tx(0){
            T_target.setIdentity();
            T_source.setIdentity();
        };

        MaxStabbingResult &operator = (const MaxStabbingResult &other){
            inlier_count = other.inlier_count;
            target_plane_idx = other.target_plane_idx;
            source_plane_idx = other.source_plane_idx;
            tx = other.tx;
            T_target = other.T_target;
            T_source = other.T_source;
            inlier_corr_idxs.assign(other.inlier_corr_idxs.begin(), other.inlier_corr_idxs.end());
            return *this;
        }

        void plot(){
            std::cout << "inlier_count: " << inlier_count << "; tx: " << tx << "; corr plane idx: " << target_plane_idx << "-" << source_plane_idx << std::endl;
        }
    };

    class LocalMapMatcher {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LocalMapMatcher();

        ~LocalMapMatcher();

        RegistrationResult Match(PointCloudT cloud_source, PointCloudT cloud_target,
                                 PointCloudT ground_source, PointCloudT ground_target,
                                 PointCloudT plane_cloud_source, PointCloudT plane_cloud_target,
                                 std::vector<pcl::ModelCoefficients> planes_target,
                                 std::vector<pcl::ModelCoefficients> planes_source);

        void Run();

        void LoadData(int loop_count, int ref_idx, int query_idx);

    private:

        std::vector<pcl::ModelCoefficients> LoadCP(const std::string& file_path);

        void GroundCoordinate();

        PointCloudTX GetGroundPlane(PointCloudTX cloud_in);

        pcl::ModelCoefficients PlaneRANSACFitting(PointCloudTX cloud_in,PointCloudTX& cloud_plane);

        void ISSExt(PointCloudTX::Ptr cloud, PointCloudTX::Ptr ISS,
                    pcl::PointIndicesPtr ISSIdx, double inlTh);

        void FPFHComp(PointCloudTX::Ptr cloud, double inlTh,
                      pcl::PointIndicesPtr ISSIdx,
                      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhOut);

        void PreprocessingCloud(PointCloudTX::Ptr cloud, Eigen::Matrix4d& T_mCent, PointCloudTX::Ptr iss,
                                pcl::PointIndicesPtr issIdx, pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh);

        void CorrComp(pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs,
                      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfht,
                      std::vector<corrTab> &corr, int MaxNOCorrPP,
                      std::vector<int> &corrNOS, std::vector<int> &corrNOT);

        Eigen::Matrix4d PlaneCoordinate();

        MaxStabbingResult PointMatching(const PointCloudTX& iss_source,
                                        const PointCloudTX& iss_target,
                                        std::vector<Eigen::Matrix4d> tmp_T_v,
                                        std::vector<corrTab> corr);

        line_interval LineBallIntersection(PointTX p_source, PointTX p_target, double eps);

        int MaxStabbing(std::vector<line_interval>& intervals, double& out_t);

        Eigen::Matrix4d GroundPlane2T(pcl::ModelCoefficients plane_model);

        Eigen::Matrix4d TwoPlane2T(pcl::ModelCoefficients ground_plane, pcl::ModelCoefficients choosed_plane);

        std::vector<Eigen::Matrix4d> GetPredefinedTransformations2Plane1Line(
                Eigen::Vector4d Pi1, Eigen::Vector4d Pi2,
                Eigen::Vector4d Nu1, Eigen::Vector4d Nu2,
                bool verbose);

// utinity functions

        pcl::PointCloud<pcl::PointXYZ> ChangeCloud(pcl::PointCloud<pcl::PointXYZI> cloud);

        pcl::ModelCoefficients TransformPlane(pcl::ModelCoefficients plane, Eigen::Matrix4d T);

        Eigen::Vector4d PlaneModel2Eigen(pcl::ModelCoefficients plane);

        Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R);

        template <typename Derived>
        Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr);

        RegistrationResult ICPRegistration(PointCloudTX ref_cloud, PointCloudTX query_cloud, Eigen::Matrix4f init_guess);

        void ErrorEvaluation(Eigen::Matrix4d GT, Eigen::Matrix4d Mes);

    private:

        PointCloudTX::Ptr cloud_source_;
        PointCloudTX::Ptr cloud_target_;
        PointCloudTX::Ptr ground_source_;
        PointCloudTX::Ptr ground_target_;
        PointCloudTX::Ptr plane_cloud_source_;
        PointCloudTX::Ptr plane_cloud_target_;
        PointCloudTX::Ptr all_source_;
        PointCloudTX::Ptr all_target_;

        std::string OUTPUT_PATH_;

        Eigen::Matrix4d T_ground_target_;
        Eigen::Matrix4d T_ground_source_;

        std::vector<pcl::ModelCoefficients> planes_target_;
        std::vector<pcl::ModelCoefficients> planes_source_;

        pcl::ModelCoefficients ground_plane_target_;
        pcl::ModelCoefficients ground_plane_source_;

        double inlTh_;
        int maxCorr_;
        double tx_min_;
        double tx_max_;
        double epsilon_;

        bool showCloud_;

        Eigen::Matrix4d mat_GT_;
        bool test_accuracy_;

    };

}



#endif //GP_LIO_LOCALMAPMATCHER_H
