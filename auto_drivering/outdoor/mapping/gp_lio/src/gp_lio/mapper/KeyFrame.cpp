//
// Created by wchen on 2019/12/3.
//

#include "gp_lio/mapper/KeyFrame.h"


namespace gp_lio{

    KeyFrame::KeyFrame(){
        descriptor_.calculated_  = false;
        descriptor_.descriptor_.resize(256, 0.0);
        feature_cloud_.clear();
        index_ = -1;
        loop_index_ = -1;
        sequence_ = 1;
        has_loop = false;
        has_prior = false;
    }

    KeyFrame::KeyFrame(gp_lio::Frame frame) {

        this->state_.swap(frame.state_);
        this->cloud_feature_ = frame.cloud_feature_;
        this->surfPairsGroup_ = frame.surfPairsGroup_;
        this->groundPairsGroup_ = frame.groundPairsGroup_;
        this->edgePairsGroup_ = frame.edgePairsGroup_;
        this->surfPairsGroupForward_ = frame.surfPairsGroupForward_;
        this->edgePairsGroupForward_ = frame.edgePairsGroupForward_;
        this->planeCorPairsGroup_ = frame.planeCorPairsGroup_;

        descriptor_.calculated_  = false;
        descriptor_.descriptor_.resize(256, 0.0);
        feature_cloud_.clear();
        index_ = -1;
        loop_index_ = -1;
        sequence_ = 1;
        has_loop = false;
    }

    KeyFrame::~KeyFrame() {

    }

    pcl::PointCloud<pcl::PointXYZ> KeyFrame::GetTransformedFeatureCloud(ExternalParameters& extrinsic_parameters) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();
        pcl::transformPointCloud(feature_cloud_, *transformedCloud, trans);

        return *transformedCloud;

    }

    pcl::PointCloud<pcl::PointXYZI> KeyFrame::GetTransformedAllCloud(ExternalParameters& extrinsic_parameters) {

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr allCloud(new pcl::PointCloud<pcl::PointXYZI>());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        *allCloud += cloud_feature_.less_edge_feature_;
        *allCloud += cloud_feature_.less_surf_feature_;
        *allCloud += cloud_feature_.ground_feature_;
        // *allCloud += cloud_feature_.outlier_feature_;
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();
        pcl::transformPointCloud(*allCloud, *transformedCloud, trans);

        return *transformedCloud;

    }

    void KeyFrame::ConstructFeatureCloud(){

        feature_cloud_.clear();

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (auto & i : cloud_feature_.less_surf_feature_) {
            pcl::PointXYZ point;
            point.x = i.x;
            point.y = i.y;
            point.z = i.z;
            cloud->points.emplace_back(point);
        }

        for (auto & i : cloud_feature_.less_edge_feature_) {
            pcl::PointXYZ point;
            point.x = i.x;
            point.y = i.y;
            point.z = i.z;
            cloud->points.emplace_back(point);
        }

        pcl::RandomSample<pcl::PointXYZ> sampler;
        // std::cout << "ConstructFeatureCloud: " << cloud->points.size() << " ";

        if (cloud->points.size() >= 4096){

            sampler.setInputCloud(cloud);
            sampler.setSample(4096);
            sampler.filter(feature_cloud_);

        }else{

            pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud (new pcl::PointCloud<pcl::PointXYZ>);

            for (auto & i : cloud_feature_.outlier_feature_) {

                pcl::PointXYZ point;
                point.x = i.x;
                point.y = i.y;
                point.z = i.z;
                outlier_cloud->points.emplace_back(point);
            }

            if (outlier_cloud->points.size() + cloud->points.size() < 4096){

                for (auto & i : cloud_feature_.ground_feature_) {

                    pcl::PointXYZ point;
                    point.x = i.x;
                    point.y = i.y;
                    point.z = i.z;
                    outlier_cloud->points.emplace_back(point);
                }

            }
            std::cout << (4096 - cloud->points.size()) << std::endl;

            sampler.setInputCloud(outlier_cloud);
            sampler.setSample(4096 - cloud->points.size());
            sampler.filter(feature_cloud_);

            feature_cloud_ = feature_cloud_ + *cloud;
        }
    }

}