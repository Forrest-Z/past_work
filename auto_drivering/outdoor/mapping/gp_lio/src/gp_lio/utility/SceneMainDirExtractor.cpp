//
// Created by wchen on 2020/5/29.
//

#include "gp_lio/utility/SceneMainDirExtractor.h"

namespace gp_lio{

    SceneMainDirExtractor::SceneMainDirExtractor() {

        eps_ = 1.0/180.0*M_PI;
        min_points_ = 5;
        cluster_idx_ = -1;

        kdTree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
//        kdTree_->setEpsilon(eps_);
//        kdTree_->setMinPts(min_points_);

        point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }

    SceneMainDirExtractor::~SceneMainDirExtractor() {

    }

    void SceneMainDirExtractor::SetCorrespondencePairs(gp_lio::FeatureCorPairsGroup& surfCorPairsGroup) {


        clusters_.clear();
        points_vec_.clear();
        point_clusters_.clear();
        cor_pairs_clusters_.clear();
        outlier_pairs_.clear();

        surfCorPairsGroup_ = surfCorPairsGroup;
        points_vec_.resize(surfCorPairsGroup_.size());
        point_cloud_->clear();

        for (int i = 0; i < surfCorPairsGroup_.size(); ++i) {

            ClusterPoint point;
            point.point_.x = surfCorPairsGroup_.at(i).dir_vec_.x();
            point.point_.y = surfCorPairsGroup_.at(i).dir_vec_.y();
            point.point_.z = surfCorPairsGroup_.at(i).dir_vec_.z();

            points_vec_.at(i) = point;
            point_cloud_->push_back(point.point_);

        }
        kdTree_->setInputCloud(point_cloud_);

        FindNeighbor();
        DBscanClustering();


    }

    std::vector<std::vector<FeatureCorPairs>> SceneMainDirExtractor::GetClusters() {
        return cor_pairs_clusters_;
    }

    std::vector<std::vector<pcl::PointXYZ>> SceneMainDirExtractor::GetClusterPoints() {
        return point_clusters_;
    }

    FeatureCorPairsGroup SceneMainDirExtractor::GetOutlierPairs() {
        return outlier_pairs_;
    }

    void SceneMainDirExtractor::FindNeighbor() {

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int i = 0; i < points_vec_.size(); ++i) {

            pointSearchInd.resize(min_points_);
            pointSearchSqDis.resize(min_points_);

            pcl::PointXYZ point = points_vec_.at(i).point_;
            kdTree_->nearestKSearch(point, min_points_, pointSearchInd, pointSearchSqDis);
            if(pointSearchSqDis.at(min_points_-1) > eps_)
                continue;
            else{
                points_vec_.at(i).neighbor_idxs_ = pointSearchInd;
                points_vec_.at(i).neighbor_dis_ = pointSearchSqDis;
            }
        }
    }

    void SceneMainDirExtractor::DBscanClustering() {

        for (int i = 0; i < points_vec_.size(); ++i) {

            if(points_vec_.at(i).cluster_ != -1) continue;

            if(!points_vec_.at(i).neighbor_idxs_.empty()){
                DFS(i, ++cluster_idx_);
            } else{
                points_vec_.at(i).cluster_ = -2; // outlier
            }
        }
        clusters_.resize(cluster_idx_+1);
        cor_pairs_clusters_.resize(cluster_idx_+1);
        point_clusters_.resize(cluster_idx_+1);
        for (int j = 0; j < points_vec_.size(); ++j) {

            if(points_vec_.at(j).cluster_ != -2){

                clusters_.at(points_vec_.at(j).cluster_).push_back(j);
                cor_pairs_clusters_.at(points_vec_.at(j).cluster_).push_back(surfCorPairsGroup_.at(j));
                point_clusters_.at(points_vec_.at(j).cluster_).push_back(points_vec_.at(j).point_);

            } else
                outlier_pairs_.push_back(surfCorPairsGroup_.at(j));

        }
    }

    void SceneMainDirExtractor::DFS(int idx, int cluster_idx) {

        points_vec_.at(idx).cluster_ = cluster_idx;
        if(points_vec_.at(idx).neighbor_idxs_.empty()) return;


        for (auto& next:points_vec_.at(idx).neighbor_idxs_){
            if(points_vec_.at(next).cluster_ != -1) continue;
            DFS(next, cluster_idx);
        }
    }

}