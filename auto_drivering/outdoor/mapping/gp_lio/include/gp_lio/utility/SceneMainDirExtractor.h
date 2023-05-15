//
// Created by wchen on 2020/5/29.
//

#ifndef GP_LIO_SCENEMAINDIREXTRACTOR_H
#define GP_LIO_SCENEMAINDIREXTRACTOR_H

#include <vector>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "gp_lio/utility/Utility.h"



// This class extract the main directions of the scene from the associated feature pairs,
// and complete the incremental plane fitting.

namespace gp_lio{

    struct ClusterPoint{

        pcl::PointXYZ point_;
        std::vector<int> neighbor_idxs_;
        std::vector<float> neighbor_dis_;
        int cluster_=-1; // -1 unclusterred ; -2 outlier

    };

    class SceneMainDirExtractor {

    public:
        SceneMainDirExtractor();

        ~SceneMainDirExtractor();

        void SetCorrespondencePairs(FeatureCorPairsGroup &surfCorPairsGroup);

        std::vector<std::vector<FeatureCorPairs>> GetClusters();

        std::vector<std::vector<pcl::PointXYZ>> GetClusterPoints();

        FeatureCorPairsGroup GetOutlierPairs();

    public:

        FeatureCorPairsGroup surfCorPairsGroup_;

    private:

        void FindNeighbor();

        void DBscanClustering();

        void DFS(int idx, int cluster_idx);

        std::vector<ClusterPoint> points_vec_;

        std::vector<std::vector<int>> clusters_;
        std::vector<std::vector<pcl::PointXYZ>> point_clusters_;
        std::vector<std::vector<FeatureCorPairs>> cor_pairs_clusters_;
        FeatureCorPairsGroup outlier_pairs_;

        float eps_;
        int min_points_;
        int cluster_idx_;

        pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdTree_;

    };

}



#endif //GP_LIO_SCENEMAINDIREXTRACTOR_H
