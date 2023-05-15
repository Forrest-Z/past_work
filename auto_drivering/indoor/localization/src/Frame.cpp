//
// Created by wchen on 2019/12/3.
//

#include "Frame.h"


namespace gp_lio{

    Frame::Frame() {

        state_.clear();

    }

    Frame::Frame(State& state, CloudFeature& cloud_feature) {

        state_ = state;
        cloud_feature_ = cloud_feature;
    }

    Frame::~Frame() {

    }



    PointCloudT Frame::GetTransformedPlaneCloud(ExternalParameters &extrinsic_parameters) {

        PointCloudT::Ptr transformedPlaneCloud(new PointCloudT());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();

        pcl::transformPointCloud(cloud_feature_.plane_cloud_, *transformedPlaneCloud, trans);

        return *transformedPlaneCloud;
    }


    PointCloudT Frame::GetTransformedGroundCloud(ExternalParameters& extrinsic_parameters) {

        PointCloudT::Ptr transformedGroundCloud(new PointCloudT());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();

        pcl::transformPointCloud(cloud_feature_.ground_feature_, *transformedGroundCloud, trans);

        return *transformedGroundCloud;

    }

    PointCloudT Frame::GetTransformedEdgeCloud(ExternalParameters& extrinsic_parameters) {

//        PointCloudT::Ptr transformedCloud(new PointCloudT());
        PointCloudT::Ptr transformedLessCloud(new PointCloudT());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();
//        pcl::transformPointCloud(cloud_feature_.edge_feature_, *transformedCloud, trans);

        pcl::transformPointCloud(cloud_feature_.less_edge_feature_, *transformedLessCloud, trans);

//        *transformedCloud = *transformedLessCloud;

        return *transformedLessCloud;
    }

    PointCloudT Frame::GetTransformedSurfCloud(ExternalParameters& extrinsic_parameters) {

//        PointCloudT::Ptr transformedCloud(new PointCloudT());
        PointCloudT::Ptr transformedLessCloud(new PointCloudT());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state_.getTransformation() * extrinsic_parameters.getTransform();
//        pcl::transformPointCloud(cloud_feature_.surf_feature_, *transformedCloud, trans);
        pcl::transformPointCloud(cloud_feature_.less_surf_feature_, *transformedLessCloud, trans);

//        *transformedCloud = *transformedLessCloud;

        return *transformedLessCloud;

    }

    std::vector<PlaneFeature> Frame::GetTransformedPlane(gp_lio::ExternalParameters &extrinsic_parameters) {

        if(cloud_feature_.plane_feature_.empty())
            return std::vector<PlaneFeature>();

        std::vector<PlaneFeature> planes;
        Eigen::Matrix4d pose = state_.getTransformation();
        Eigen::Matrix4d rel_T = pose * extrinsic_parameters.getTransform();
        for (int i = 0; i < cloud_feature_.plane_feature_.size(); ++i) {
            auto plane = cloud_feature_.plane_feature_.at(i);
            plane.Transform(rel_T);
            planes.emplace_back(plane);
        }

        return planes;

    }

    void Frame::SetAssociationPairsGroup(FeatureCorPairsGroup &edgePairs,
                                         FeatureCorPairsGroup &surfPairs) {

        edgePairsGroup_ = edgePairs;
        surfPairsGroup_ = surfPairs;

    }

    void Frame::SetAssociationGroundPairsGroup(gp_lio::FeatureCorPairsGroup &groundPairs) {

        groundPairsGroup_ = groundPairs;

    }

    void Frame::SetAssociationPairsGroupForward(gp_lio::FeatureCorPairsGroup &edgePairs,
                                                gp_lio::FeatureCorPairsGroup &surfPairs) {

        edgePairsGroupForward_ = edgePairs;
        surfPairsGroupForward_ = surfPairs;

    }

    void Frame::SetAssociationPlaneGroup(gp_lio::PlaneCorPairsGroup planePairGroup) {
        planeCorPairsGroup_ = planePairGroup;
    }

    void Frame::SetCloud(gp_lio::CloudFeature &cloud_feature) {

        cloud_feature_.clear();
        cloud_feature_ = cloud_feature;

    }

    void Frame::clear() {

        state_.clear();
        cloud_feature_.clear();
        edgePairsGroup_.clear();
        surfPairsGroup_.clear();
        groundPairsGroup_.clear();
        edgePairsGroupForward_.clear();
        surfPairsGroupForward_.clear();
        planeCorPairsGroup_.clear();

    }

    void Frame::swap(gp_lio::Frame frame) {
        this->clear();
        this->state_.swap(frame.state_);
        this->cloud_feature_ = frame.cloud_feature_;
        this->surfPairsGroup_ = frame.surfPairsGroup_;
        this->groundPairsGroup_ = frame.groundPairsGroup_;
        this->edgePairsGroup_ = frame.edgePairsGroup_;
        this->surfPairsGroupForward_ = frame.surfPairsGroupForward_;
        this->edgePairsGroupForward_ = frame.edgePairsGroupForward_;
        this->planeCorPairsGroup_ = frame.planeCorPairsGroup_;
    }

}