//
// Created by wchen on 2019/12/3.
//
// Input: undistorted point cloud;
// Output: feature cloud(planar, edge, ground)
// Main algorithm: ref to lego_loam

#ifndef SRC_LIDARFEATURE_H
#define SRC_LIDARFEATURE_H

#include <string>
#include <fstream>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Eigenvalues>

// ceres
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/loss_function.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>

#include "gp_lio/utility/Utility.h"
#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/estimator/Parameters.h"

namespace gp_lio
{
    // 1 - residual, 3 - CP
    class PlaneFittingCostFunction: public ceres::SizedCostFunction<1, 3>{

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PlaneFittingCostFunction() = delete;

        PlaneFittingCostFunction(const Eigen::Vector3f point, const float & point_std){
            point_ = Eigen::Vector3d(point.x(), point.y(), point.z());
            covariance_.setIdentity();
            covariance_ = covariance_ * double(point_std * point_std);
        };

        virtual ~PlaneFittingCostFunction() {}

        virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const{

            Eigen::Vector3d cp(parameters[0][0], parameters[0][1], parameters[0][2]);

            double residual = cp.transpose()/cp.norm() * point_ - cp.norm();

            double weight = (cp.transpose()/cp.norm()) * covariance_ * (cp/cp.norm());

//            double w = std::sqrt(1.0/weight);
            double w = 1.0;
//            ROS_DEBUG_STREAM("w: " << w << " cp: " << cp.transpose());

            residuals[0] = w * residual;

            if(jacobians != NULL){

                if(jacobians[0] != NULL){

                    Eigen::Vector3d jacobian_cp = point_.transpose() / cp.norm() - (point_.transpose() * cp) * cp.transpose() / std::pow(cp.norm(), 3) - cp.transpose() / cp.norm();

                    jacobians[0][0] = jacobian_cp.x();
                    jacobians[0][1] = jacobian_cp.y();
                    jacobians[0][2] = jacobian_cp.z();

                    //                    ROS_DEBUG_STREAM("jacobian " << jacobian_cp);
                }

            }
            return true;

        };

    private:
        Eigen::Vector3d point_;
        Eigen::Matrix3d covariance_;

    };


    class LidarFeature
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        LidarFeature();

        ~LidarFeature();

        CloudFeature PutinCloud(CloudMeasurement &cloud_measurement, PointCloudT &cloud_last, Eigen::Matrix4d &transform_matrix);

        PointCloudT cloud_;

        PointCloudT last_cloud_;
        
    private:
        void Init();

        // image projection
        void RangeImageProcess();

        void dynamic_filter(PointCloudT &point_cloud1, PointCloudT &point_cloud2, Eigen::Matrix4d &transform_matrix, float threshold1, float threshold2, float threshold3, int k); 

        void FindStartEndAngle();

        void ProjectPointCloud();

        void GroundPointsExtraction();

        void CloudSegmentation();

        void labelComponents(int row, int col);

        // feature extraction
        void FeatureExtractionProcess();

        void AssignTimestampToPoints();

        void CalculateSmoothness();

        void MarkOccludedPoints();

        void ExtractFeatures();

        void ExtractPlane();

        void CPPlaneRefine(PlaneFeature & planeFeature);

        void ConstructCloudFeature();

        void FreeMemory();

    private:
        // PointCloudT cloud_;
        double received_time_;

        PointCloudT::Ptr fullCloud_;     // projected cloud, intensity represents (rowIdn + columnIdn/10000.0) rowIdn is very useful in feature association
        PointCloudT::Ptr fullInfoCloud_; // same as fullCloud_, but intensity represents range
        PointCloudT::Ptr groundCloud_;
        PointCloudT::Ptr outlierCloud_;
        PointCloudT::Ptr segmentedCloud_; // with ground; after AssignTimestampToPoints(), point.intensity = int(segmentedCloud_->points[i].intensity) + SCAN_DURATION * oriRatio;
        std::vector<int> segmentedCloudIdx_; //16 1800

        std::vector<PlaneFeature> planeFeature_;
        PointCloudT::Ptr planeCloud_;

        // range image for cloud clustering
        cv::Mat rangeMat_;
        cv::Mat groundMat_; // -1: no check; 1: ground; -1: no ground
        cv::Mat labelMat_;  // label matrix for segmentaiton marking
        cv::Mat featureTypeMat_; // represent feature type; 0: outlier; -1 less edge feature; 1 less surf feature

        // segmentation
        CloudInfo segCloudInfo_;
        std::vector<std::pair<int8_t, int8_t>> neighborIterator_; // neighbor iterator for segmentaiton process
        uint16_t *allPushedIndX_;                                 // array for tracking points of a segmented object
        uint16_t *allPushedIndY_;
        uint16_t *queueIndX_; // array for breadth-first search process of segmentation
        uint16_t *queueIndY_;
        int labelCount_;

        // feature extraction
        std::vector<smoothness_t> cloudSmoothness_;
        std::vector<float> cloudCurvature_;
        std::vector<int> cloudNeighborPicked_;
        std::vector<int> cloudLabel_;

        PointCloudT::Ptr cornerPointsSharp_;
        PointCloudT::Ptr cornerPointsLessSharp_;
        PointCloudT::Ptr surfPointsFlat_;
        PointCloudT::Ptr surfPointsLessFlat_;

        PointCloudT::Ptr surfPointsLessFlatScan_;
        PointCloudT::Ptr surfPointsLessFlatScanDS_;
        pcl::VoxelGrid<PointT> downSizeFilter_;

        CloudFeature extracted_feature_;

    };
} // namespace gp_lio

#endif //SRC_LIDARFEATURE_H
