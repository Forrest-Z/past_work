//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_LOCALMAPPER_H
#define SRC_LOCALMAPPER_H

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid_label.h>

#include <Eigen/Eigenvalues>

#include <string>

#include "gp_lio/utility/Utility.h"
#include "gp_lio/estimator/Parameters.h"
#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/mapper/Frame.h"
#include "gp_lio/utility/SceneMainDirExtractor.h"

namespace gp_lio
{

class LocalMapper
{

public:

    LocalMapper();

    ~LocalMapper();

    void Clear();

    void AddNewCloudFeature(CloudFeature &cloud_feature, int frame_count);

    void DataAssociation();

    void DataAssociationAdjacent();

    void DataAssociationAdjacentGround();

    void PlaneAssociationAdjacent();

    MarginalizationFlag CheckMarginalizationType(int &frame_count, bool is_moving= true);

    void SlideWindow(MarginalizationFlag marginalization_flag);

    void Add2LocalMap(PointCloudT edge, PointCloudT surf, PointCloudT ground, std::vector<PlaneFeature> planes, State state);

    PointCloudT GetLocalMap();

    void AddClosedLocalMap(PointCloudT edge, PointCloudT surf, PointCloudT ground, std::vector<PlaneFeature> planes);

    void AssembleLocalMap();

private:

    PointT PointAssociatedToMap(PointT &pi, State &state);

    PointCloudT TransformPointCloud(PointCloudT cloud_in, State state);

public:
    Frame frame_in_window_[WINDOW_SIZE + 1];
    ExternalParameters extrinsic_parameters_;

private:
    pcl::VoxelGrid<PointT> downSize_filter_edge_map_;
    pcl::VoxelGrid<PointT> downSize_filter_surf_map_;
    pcl::VoxelGrid<PointT> downSize_filter_ground_map_;
    pcl::KdTreeFLANN<PointT>::Ptr kdTree_edge_map_;
    pcl::KdTreeFLANN<PointT>::Ptr kdTree_surf_map_;
    pcl::KdTreeFLANN<PointT>::Ptr kdTree_ground_map_;

    std::deque<PointCloudT> edge_map_deque_, surf_map_deque_, ground_map_deque_;
    PointCloudT::Ptr edge_map_DS_, surf_map_DS_, ground_map_DS_;

    PointCloudT::Ptr closed_edge_map_, closed_surf_map_, closed_ground_map_;
    std::vector<PlaneFeature> closed_plane_feature_;

    std::deque<std::vector<PlaneFeature>> plane_feature_deque_;

    PointCloudT::Ptr local_map_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::deque<State> pose_deque_;
};
} // namespace gp_lio

#endif //SRC_LOCALMAPPER_H
