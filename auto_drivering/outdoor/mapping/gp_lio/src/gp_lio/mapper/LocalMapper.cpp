//
// Created by wchen on 2019/12/3.
//

#include <gp_lio/mapper/LocalMapper.h>
#include "gp_lio/mapper/LocalMapper.h"

namespace gp_lio
{

    LocalMapper::LocalMapper()
    {

        downSize_filter_edge_map_.setLeafSize(DOWNSIZE_Th, DOWNSIZE_Th, DOWNSIZE_Th);
        downSize_filter_surf_map_.setLeafSize(DOWNSIZE_Th, DOWNSIZE_Th, DOWNSIZE_Th);
        downSize_filter_ground_map_.setLeafSize(DOWNSIZE_Th, DOWNSIZE_Th, DOWNSIZE_Th);

        kdTree_edge_map_.reset(new pcl::KdTreeFLANN<PointT>());
        kdTree_surf_map_.reset(new pcl::KdTreeFLANN<PointT>());
        kdTree_ground_map_.reset(new pcl::KdTreeFLANN<PointT>());

        edge_map_DS_.reset(new PointCloudT());
        surf_map_DS_.reset(new PointCloudT());
        ground_map_DS_.reset(new PointCloudT());
        local_map_.reset(new PointCloudT());

        closed_edge_map_.reset(new PointCloudT());
        closed_surf_map_.reset(new PointCloudT());
        closed_ground_map_.reset(new PointCloudT());
    }

    LocalMapper::~LocalMapper()
    {
    }

    void LocalMapper::Clear()
    {

        for (int i = 0; i <= WINDOW_SIZE; ++i)
        {

            frame_in_window_[i].clear();
        }
    }

    void LocalMapper::AddNewCloudFeature(gp_lio::CloudFeature &cloud_feature, int frame_count)
    {

        frame_in_window_[frame_count].SetCloud(cloud_feature);
        is_still = 1;
    }

    void LocalMapper::DataAssociation()
    {

        // TODO: This method does not work well for some reasons:
        // TODO 1. The first five keyframes are set fixed for assemble local map. However, the sparse sensing character makes
        // TODO the association difficult. Because there are not enough points are collected from the same structures;
        // TODO 2. Outlier rejection steps are not included in the process. Wrong data pairs result in bad optimization results.

        // In Liu Ming's work (only planar feature is usable), the first frame in the window is treated as reference frame.
        // N frames before it or M frames after it are transformed to the reference frame,
        // and the features of the former frames are treated as measurements in pivot frame for making cloud denser.
        // For each feature, find some nearest features which can be represented as a plane.
        // Then, the point-to-plane residual is applied in the LiDAR factor. The feature is transformed by the
        // {w}{pivot}T and {w}{k}T, the plane function does not contain any states to be optimized which is not so good
        // and time-consuming.
        // The association method is similar with the mapping part in LOAM, and only use the plane feature.

        // In LOAM, we find edge line as the correspondence for an edge feature,
        // a planar patch as the correspondence for a planar feature.
        // In odometry part, the feature association happens between two adjacent frames (10Hz),
        // By assuming the constant angular and linear velocities motion model, linear interpolation is applied to
        // process each point measured in different time. So after the odometry step, the cloud is undistorted.
        // But how to get the initial guess? -- copy the last optimized result of relative transform.
        // In mapping part, the new cloud Q is represented in the {L_t+2} frame, and
        // from the odometry part, we already know the transform between {L_t+1} and {L_t+2} (to be optimized)
        // and the transform from {L_t+1} to {w}. For each feature in Q, we find some nearest features from map (has been
        // cropped by intersection relationship), and form correspondences for edge or planar features. For edge features,
        // an edge line is extracted in this nearest cloud by eigenvalue analysis. For planar features, similar method is
        // applied. In order to establish the same correspondences' residual function with the former odometry, choosing
        // two points from line or three points from plane is applied.

        // how to do in this gp-lio system?
        // If we have a good extrinsic parameters, the results of the imu integration can be used as the initial
        // guess between two lidar clouds. Then the data association can be performed relatively less time. However,
        // extrinsic parameters is part of the optimized states.
        // And in the sliding window, every Key frames would be moved from the end of the window to the begin.
        // So the related states have been optimized several times (depends on the window size).
        // I guess it's enough for them. So I choose to fix the states in the former part of the window.
        // For the data association, similar with the mapping part in LOAM, for each feature, we find some nearest features
        // from the local map constructed by the fixed states. we also use point-to-line residual and point-to-plane
        // residual as lidar factors.
        AssembleLocalMap();

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int frame_i = FIXED_SIZE; frame_i < WINDOW_SIZE + 1; ++frame_i) {
            FeatureCorPairsGroup newEdgeCorGroup;
            Frame frame_1 = frame_in_window_[frame_i];
            Eigen::Matrix4d pose_1 = frame_1.state_.getTransformation();
            Eigen::Matrix4d rel_T = pose_1 * extrinsic_parameters_.getTransform();

            if (USE_EDGE_FACTOR) {

                PointCloudT::Ptr edge(new PointCloudT);
                pcl::transformPointCloud(frame_1.cloud_feature_.less_edge_feature_, *edge, rel_T);

                int edgeFeatureSize = edge->size();
                newEdgeCorGroup.reserve(edgeFeatureSize);

                for (int i = 0; i < edgeFeatureSize; ++i) {

                    //find nearest five points, and check its eigenvalues and eigenvectors
                    PointT pointSel = edge->at(i);
                    kdTree_edge_map_->nearestKSearch(pointSel, NEAREST_POINTS_NUM, pointSearchInd, pointSearchSqDis);

                    if (pointSearchSqDis[NEAREST_POINTS_NUM - 1] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
                        std::vector<Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for (int j = 0; j < NEAREST_POINTS_NUM; j++) {
                            Eigen::Vector3d tmp(
                                    edge_map_DS_->points[pointSearchInd[j]].x,
                                    edge_map_DS_->points[pointSearchInd[j]].y,
                                    edge_map_DS_->points[pointSearchInd[j]].z);
                            center = center + tmp;
                            nearCorners.push_back(tmp);
                        }
                        center = center / double(NEAREST_POINTS_NUM);

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for (int j = 0; j < NEAREST_POINTS_NUM; j++) {
                            Eigen::Matrix<double, 3, 1> tmpZeroMean =
                                    nearCorners[j] - center;
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                        // if is indeed line feature
                        // note Eigen library sort eigenvalues in increasing order
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d last_point_a, last_point_b;
                            last_point_a = 0.1 * unit_direction + point_on_line;
                            last_point_b = -0.1 * unit_direction + point_on_line;

                            PointT tripod1 = PointT(
                                    {static_cast<float>(last_point_a.x()), static_cast<float>(last_point_a.y()),
                                     static_cast<float>(last_point_a.z())});
                            PointT tripod2 = PointT(
                                    {static_cast<float>(last_point_b.x()), static_cast<float>(last_point_b.y()),
                                     static_cast<float>(last_point_b.z())});

                            FeatureCorPairs edgeCor;
                            FeatureType featureType = FeatureType::Edge;
                            edgeCor.setFeatureType(featureType);
                            edgeCor.feature_ = frame_1.cloud_feature_.less_edge_feature_.points.at(i);
                            edgeCor.corresponding_points_.push_back(tripod1);
                            edgeCor.corresponding_points_.push_back(tripod2);
                            edgeCor.dir_vec_ = unit_direction;

                            newEdgeCorGroup.push_back(edgeCor);
                        }
                    }
                }
            }

            FeatureCorPairsGroup newSurfCorGroup;
            if (USE_SURF_FACTOR) {

                PointCloudT::Ptr surf(new PointCloudT);
                pcl::transformPointCloud(frame_1.cloud_feature_.less_surf_feature_, *surf, rel_T);
                int surfFeatureSize = surf->size();
                newSurfCorGroup.reserve(surfFeatureSize);

                for (int j = 0; j < surfFeatureSize; ++j) {

                    //find nearest five points, and check its eigenvalues and eigenvectors
                    PointT pointSel = surf->at(j);
                    kdTree_surf_map_->nearestKSearch(pointSel, NEAREST_POINTS_NUM, pointSearchInd, pointSearchSqDis);

                    if (pointSearchSqDis[NEAREST_POINTS_NUM - 1] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
                        Eigen::Matrix<double, NEAREST_POINTS_NUM, 3> matA0;
                        Eigen::Matrix<double, NEAREST_POINTS_NUM, 1> matB0 =
                                -1 * Eigen::Matrix<double, 5, 1>::Ones();
                        for (int l = 0; l < NEAREST_POINTS_NUM; l++) {
                            matA0(l, 0) = surf_map_DS_->points[pointSearchInd[l]].x;
                            matA0(l, 1) = surf_map_DS_->points[pointSearchInd[l]].y;
                            matA0(l, 2) = surf_map_DS_->points[pointSearchInd[l]].z;
                            // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
                            // 2));
                        }
                        // find the norm of plane
                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        // Here n(pa, pb, pc) is unit norm of plane
                        bool planeValid = true;
                        for (int k = 0; k < NEAREST_POINTS_NUM; k++) {
                            // if OX * n > 0.2, then plane is not fit well
                            if (fabs(norm(0) *
                                     surf_map_DS_->points[pointSearchInd[k]].x +
                                     norm(1) *
                                     surf_map_DS_->points[pointSearchInd[k]].y +
                                     norm(2) *
                                     surf_map_DS_->points[pointSearchInd[k]].z +
                                     negative_OA_dot_norm) > 0.02) {
                                planeValid = false;
                                break;
                            }
                        }

                        if (planeValid) {
                            Eigen::Vector3d last_point_a(
                                    surf_map_DS_->points[pointSearchInd[0]].x,
                                    surf_map_DS_->points[pointSearchInd[0]].y,
                                    surf_map_DS_->points[pointSearchInd[0]].z);
                            Eigen::Vector3d last_point_b(
                                    surf_map_DS_->points[pointSearchInd[2]].x,
                                    surf_map_DS_->points[pointSearchInd[2]].y,
                                    surf_map_DS_->points[pointSearchInd[2]].z);
                            Eigen::Vector3d last_point_c(
                                    surf_map_DS_->points[pointSearchInd[4]].x,
                                    surf_map_DS_->points[pointSearchInd[4]].y,
                                    surf_map_DS_->points[pointSearchInd[4]].z);

                            Eigen::Vector3d center = (last_point_a + last_point_b + last_point_c) / 3.0;

                            FeatureCorPairs surfCor;
                            FeatureType featureType = FeatureType::Surf;
                            surfCor.setFeatureType(featureType);
                            surfCor.feature_ = frame_1.cloud_feature_.less_surf_feature_.points.at(j);
                            surfCor.corresponding_points_.push_back(surf_map_DS_->points[pointSearchInd[0]]);
                            surfCor.corresponding_points_.push_back(surf_map_DS_->points[pointSearchInd[2]]);
                            surfCor.corresponding_points_.push_back(surf_map_DS_->points[pointSearchInd[4]]);
                            surfCor.dir_vec_ = norm;

                            surfCor.corresponding_center_ = PointT({static_cast<float>(center.x()),
                                                                    static_cast<float>(center.y()),
                                                                    static_cast<float>(center.z())});
                            surfCor.distance_ = 0;

                            newSurfCorGroup.push_back(surfCor);
                        }

                    }
                }
            }

            frame_in_window_[frame_i].SetAssociationPairsGroup(newEdgeCorGroup, newSurfCorGroup);

//            if (USE_GROUND_FACTOR) {
//
//                FeatureCorPairsGroup newGroundCorGroup;
//
//                // find ground pairs
//                PointCloudT::Ptr frame_1_ground(new PointCloudT);
//                pcl::transformPointCloud(frame_1.cloud_feature_.ground_feature_, *frame_1_ground, rel_T);
//
//                newGroundCorGroup.reserve(frame_1_ground->points.size());
//
//                for (int l = 0; l < frame_1_ground->points.size(); ++l) {
//
//                    PointT pointSel = frame_1_ground->points.at(l);
//                    kdTree_surf_map_->nearestKSearch(pointSel, NEAREST_POINTS_NUM, pointSearchInd, pointSearchSqDis);
//
//                    bool planeValid = false;
//
//                    Eigen::Matrix<double, NEAREST_POINTS_NUM, 3> matA0;
//                    Eigen::Matrix<double, NEAREST_POINTS_NUM, 1> matB0 =
//                            -1 * Eigen::Matrix<double, 5, 1>::Ones();
//                    if (pointSearchSqDis[NEAREST_POINTS_NUM - 1] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
//                        for (int j = 0; j < NEAREST_POINTS_NUM; j++) {
//                            matA0(j, 0) = ground_map_DS_->points[pointSearchInd[j]].x;
//                            matA0(j, 1) = ground_map_DS_->points[pointSearchInd[j]].y;
//                            matA0(j, 2) = ground_map_DS_->points[pointSearchInd[j]].z;
//                            // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
//                            // 2));
//                        }
//                        // find the norm of plane
//                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
//                        double negative_OA_dot_norm = 1 / norm.norm();
//                        norm.normalize();
//
//                        // Here n(pa, pb, pc) is unit norm of plane
//                        bool planeValid = true;
//                        for (int j = 0; j < NEAREST_POINTS_NUM; j++) {
//                            // if OX * n > 0.2, then plane is not fit well
//                            if (fabs(norm(0) *
//                                     ground_map_DS_->points[pointSearchInd[j]].x +
//                                     norm(1) *
//                                     ground_map_DS_->points[pointSearchInd[j]].y +
//                                     norm(2) *
//                                     ground_map_DS_->points[pointSearchInd[j]].z +
//                                     negative_OA_dot_norm) > 0.01) {
//                                planeValid = false;
//                                break;
//                            }
//                        }
//
//                        if (planeValid) {
//                            Eigen::Vector3d last_point_a(
//                                    ground_map_DS_->points[pointSearchInd[0]].x,
//                                    ground_map_DS_->points[pointSearchInd[0]].y,
//                                    ground_map_DS_->points[pointSearchInd[0]].z);
//                            Eigen::Vector3d last_point_b(
//                                    ground_map_DS_->points[pointSearchInd[2]].x,
//                                    ground_map_DS_->points[pointSearchInd[2]].y,
//                                    ground_map_DS_->points[pointSearchInd[2]].z);
//                            Eigen::Vector3d last_point_c(
//                                    ground_map_DS_->points[pointSearchInd[4]].x,
//                                    ground_map_DS_->points[pointSearchInd[4]].y,
//                                    ground_map_DS_->points[pointSearchInd[4]].z);
//
//                            Eigen::Vector3d center = (last_point_a + last_point_b + last_point_c) / 3.0;
//
//                            FeatureCorPairs surfCor;
//                            FeatureType featureType = FeatureType::Surf;
//                            surfCor.setFeatureType(featureType);
//                            surfCor.feature_ = frame_1.cloud_feature_.ground_feature_.points.at(l);
//                            surfCor.corresponding_points_.push_back(ground_map_DS_->points[pointSearchInd[0]]);
//                            surfCor.corresponding_points_.push_back(ground_map_DS_->points[pointSearchInd[2]]);
//                            surfCor.corresponding_points_.push_back(ground_map_DS_->points[pointSearchInd[4]]);
//                            surfCor.dir_vec_ = norm;
//
//                            surfCor.corresponding_center_ = PointT({static_cast<float>(center.x()),
//                                                                    static_cast<float>(center.y()),
//                                                                    static_cast<float>(center.z())});
//                            surfCor.distance_ = 0;
//
//                            newGroundCorGroup.push_back(surfCor);
//                        }
//
//                    }
//                }
//                frame_in_window_[frame_i].SetAssociationGroundPairsGroup(newGroundCorGroup);
//            }

            // if (USE_PLANE_FACTOR) {

//                 std::vector<PlaneFeature> frame1_plane = frame_1.cloud_feature_.plane_feature_;
// //            std::cout << i << " frame plane size : " << frame1_plane.size()  << " - " << frame0_plane.size() << std::endl;

//                 PlaneCorPairsGroup planeCorPairsGroup;

//                 std::vector<PlaneFeature> local_map_plane;
//                 for (int i = 0; i < plane_feature_deque_.size(); ++i) {
//                     Eigen::Matrix4d i_pose = pose_deque_.at(i).getTransformation();
//                     Eigen::Matrix4d i_rel_T = i_pose * extrinsic_parameters_.getTransform();
//                     for (int j = 0; j < plane_feature_deque_.at(i).size(); ++j) {
//                         auto plane = plane_feature_deque_.at(i).at(j);
//                         plane.Transform(i_rel_T);
//                         local_map_plane.push_back(plane);
//                     }
//                 }
//                 if (!closed_plane_feature_.empty()){
//                     local_map_plane.clear();
//                     local_map_plane.insert(local_map_plane.end(),
//                                            closed_plane_feature_.begin(),
//                                            closed_plane_feature_.end());
//                 }

//                 for (int j = 0; j < frame1_plane.size(); j++) {

//                     frame1_plane.at(j).Transform(rel_T);

//                     // 1- find nearest closest point and the center distance along the normal vector;
// //                std::cout << j << ": ";
//                     std::vector<int> index_cp;
//                     for (int k = 0; k < local_map_plane.size(); ++k) {

//                         float cp_angle = std::acos(frame1_plane.at(j).cp_.dot(local_map_plane.at(k).cp_) /
//                                                    (frame1_plane.at(j).cp_.norm() * local_map_plane.at(k).cp_.norm())) /
//                                          M_PI * 180.0;

//                         Eigen::Vector3f center_v = frame1_plane.at(j).center_ - local_map_plane.at(k).center_;
//                         float center_distance_0 = std::abs(center_v.dot(local_map_plane.at(k).normal_));
//                         float center_distance_1 = std::abs(center_v.dot(frame1_plane.at(j).normal_));
//                         float center_distance_mean = (center_distance_0 + center_distance_1) / 2;

// //                    std::cout << cp_angle << " " << center_distance_mean << " |";
//                         if (cp_angle <= PLANE_ASSOCIATED_ANGLE || center_distance_mean <= PLANE_CLUSTER_CENTER_DISTANCE)
//                             index_cp.push_back(k);
//                     }
// //                std::cout << "| angle threshold: " << PLANE_ASSOCIATED_ANGLE  << " distance threshold: " << PLANE_CLUSTER_CENTER_DISTANCE << std::endl;

//                     // 2- find overlapping
//                     float max_overlapping_rate = 0;
//                     int max_index = 0;
// //                std::cout << j << ": ";

//                     auto line_points_1 = frame1_plane.at(j).Get2DLine();
//                     Eigen::Vector2f point_1_0 = line_points_1.at(0);
//                     Eigen::Vector2f point_1_1 = line_points_1.at(0);

//                     for (int l = 0; l < index_cp.size(); ++l) {

//                         auto line_points_0 = local_map_plane.at(index_cp.at(l)).Get2DLine();

//                         Eigen::Vector2f point_0_0 = line_points_0.at(0);
//                         Eigen::Vector2f point_0_1 = line_points_0.at(1);

//                         float intersection_angle = 0;

//                         if (point_0_0[0] <= point_1_0[0]) {

//                             if (point_1_0[0] <= point_0_1[0]) {
//                                 //intersection
//                                 if (point_1_1[0] <= point_0_1[0])
//                                     intersection_angle = std::acos(
//                                             point_1_1.dot(point_1_0) / (point_1_1.norm() * point_1_0.norm()));
//                                 else
//                                     intersection_angle = std::acos(
//                                             point_1_0.dot(point_0_1) / (point_1_0.norm() * point_0_1.norm()));
//                             }
//                         } else {
//                             if (point_1_1[0] >= point_0_0[0]) {

//                                 if (point_1_1[0] >= point_0_1[0])
//                                     intersection_angle = std::acos(
//                                             point_0_0.dot(point_0_1) / (point_0_0.norm() * point_0_1.norm()));
//                                 else
//                                     intersection_angle = std::acos(
//                                             point_0_0.dot(point_1_1) / (point_0_0.norm() * point_1_1.norm()));
//                             }
//                         }

//                         float angle_0 = std::acos(point_0_0.dot(point_0_1) / (point_0_0.norm() * point_0_1.norm()));
//                         float angle_1 = std::acos(point_1_1.dot(point_1_0) / (point_1_1.norm() * point_1_0.norm()));

//                         float rate_0 = intersection_angle / angle_0;
//                         float rate_1 = intersection_angle / angle_1;

//                         float rate = rate_0 >= rate_1 ? rate_0 : rate_1;

//                         if (rate >= max_overlapping_rate) {
//                             max_overlapping_rate = rate;
//                             max_index = index_cp.at(l);
//                         }
//                     }
// //                std::cout << std::endl;

//                     if (max_overlapping_rate >= PLANE_ASSOCIATED_OVERLAPPING_RATE) {
//                         PlaneCorPair plane_pair(local_map_plane.at(max_index).cp_,
//                                                 frame_1.cloud_feature_.plane_feature_.at(j).cp_, max_index, j);
//                         planeCorPairsGroup.push_back(plane_pair);
//                     }

//                 }

//                 frame_in_window_[frame_i].SetAssociationPlaneGroup(planeCorPairsGroup);

//                 if (SAVE_FEATURE_TMP_RESULT) {

//                     std::string plane_2d_line_path =
//                             OUTPUT_PATH + std::to_string(frame_i) + "_plane_2d_line_correspondence.txt";
//                     std::ofstream fout(plane_2d_line_path.c_str());

//                     PointCloudT cloud_0, cloud_1;
//                     for (int j = 0; j < local_map_plane.size(); ++j) {
//                         cloud_0 += local_map_plane.at(j).cloud_;

//                         auto line = local_map_plane.at(j).Get2DLine();
//                         auto cp = local_map_plane.at(j).cp_;
//                         fout << "0 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose()
//                              << std::endl;
//                     }

//                     for (int j = 0; j < frame1_plane.size(); ++j) {
//                         cloud_1 += frame1_plane.at(j).cloud_;

//                         auto line = frame1_plane.at(j).Get2DLine();
//                         auto cp = frame1_plane.at(j).cp_;
//                         fout << "1 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose()
//                              << std::endl;
//                     }

//                     fout.close();

//                     if (!cloud_0.empty()) {
//                         std::string cloud_save_file =
//                                 OUTPUT_PATH + std::to_string(frame_i) + "_correspondence_plane_cloud_0" + ".pcd";
//                         pcl::io::savePCDFileBinary(cloud_save_file, cloud_0);
//                     }
//                     if (!cloud_1.empty()) {
//                         std::string cloud_save_file =
//                                 OUTPUT_PATH + std::to_string(frame_i) + "_correspondence_plane_cloud_1" + ".pcd";
//                         pcl::io::savePCDFileBinary(cloud_save_file, cloud_1);
//                     }
//                 }

//             }
        }
    }




    void LocalMapper::DataAssociationAdjacent() {
        // find correspondences between every adjoint frames. Only fixed the oldest keyframe in the window
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int i = FIXED_SIZE; i < WINDOW_SIZE + 1; ++i) {

            Frame frame_0 = frame_in_window_[i - 1];
            Frame frame_1 = frame_in_window_[i];

            Eigen::Matrix4d pose_0 = frame_0.state_.getTransformation();
            Eigen::Matrix4d pose_1 = frame_1.state_.getTransformation();

            Eigen::Matrix4d rel_T = extrinsic_parameters_.getTransform().inverse() * pose_0.inverse() * pose_1 *
                                    extrinsic_parameters_.getTransform();
            if (frame_0.cloud_feature_.less_edge_feature_.size() < 20 ||
                frame_0.cloud_feature_.less_surf_feature_.size() <= 50)
                continue;
            kdTree_edge_map_->setInputCloud(frame_0.cloud_feature_.less_edge_feature_.makeShared());
            kdTree_surf_map_->setInputCloud(frame_0.cloud_feature_.less_surf_feature_.makeShared());

            FeatureCorPairsGroup newEdgeCorGroup;
            FeatureCorPairsGroup newSurfCorGroup;

            int pointSearchCornerInd1, pointSearchCornerInd2;
            int pointSearchSurfInd1, pointSearchSurfInd2, pointSearchSurfInd3;

            if (USE_EDGE_FACTOR) {
                // find edge pairs
                PointCloudT frame_0_edge = frame_0.cloud_feature_.less_edge_feature_;
                PointCloudT::Ptr edge(new PointCloudT);
                pcl::transformPointCloud(frame_1.cloud_feature_.less_edge_feature_, *edge, rel_T);

                newEdgeCorGroup.reserve(edge->points.size());

                for (int j = 0; j < edge->size(); ++j) {
                    PointT pointSel = edge->points.at(j);
                    kdTree_edge_map_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    if (pointSearchSqDis[4] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
                        std::vector <Eigen::Vector3d> nearCorners;
                        Eigen::Vector3d center(0, 0, 0);
                        for (int j = 0; j < 5; j++) {
                            Eigen::Vector3d tmp(
                                    frame_0_edge.points[pointSearchInd[j]].x,
                                    frame_0_edge.points[pointSearchInd[j]].y,
                                    frame_0_edge.points[pointSearchInd[j]].z);
                            center = center + tmp;
                            nearCorners.push_back(tmp);
                        }
                        center = center / 5.0;

                        Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
                        for (int j = 0; j < 5; j++) {
                            Eigen::Matrix<double, 3, 1> tmpZeroMean =
                                    nearCorners[j] - center;
                            covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
                        }

                        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

                        // if is indeed line feature
                        // note Eigen library sort eigenvalues in increasing order
                        Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
                        if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                            Eigen::Vector3d point_on_line = center;
                            Eigen::Vector3d last_point_a, last_point_b;
                            last_point_a = 0.1 * unit_direction + point_on_line;
                            last_point_b = -0.1 * unit_direction + point_on_line;

                            PointT tripod1 = PointT(
                                    {static_cast<float>(last_point_a.x()), static_cast<float>(last_point_a.y()),
                                     static_cast<float>(last_point_a.z())});
                            PointT tripod2 = PointT(
                                    {static_cast<float>(last_point_b.x()), static_cast<float>(last_point_b.y()),
                                     static_cast<float>(last_point_b.z())});

                            FeatureCorPairs edgeCor;
                            FeatureType featureType = FeatureType::Edge;
                            edgeCor.setFeatureType(featureType);
                            edgeCor.feature_ = frame_1.cloud_feature_.less_edge_feature_.points.at(j);
                            edgeCor.corresponding_points_.push_back(tripod1);
                            edgeCor.corresponding_points_.push_back(tripod2);
                            edgeCor.dir_vec_ = unit_direction;

                            newEdgeCorGroup.push_back(edgeCor);
                        }
                    }
                }
            }
            // find surf pairs
            if (USE_SURF_FACTOR) {

                PointCloudT frame_0_surf = frame_0.cloud_feature_.less_surf_feature_;
                PointCloudT::Ptr surf(new PointCloudT);
                pcl::transformPointCloud(frame_1.cloud_feature_.less_surf_feature_, *surf, rel_T);

                newSurfCorGroup.reserve(surf->points.size());

                for (int l = 0; l < surf->points.size(); ++l) {

                    PointT pointSel = surf->points.at(l);
                    kdTree_surf_map_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                    bool planeValid = false;

                    Eigen::Matrix<double, 5, 3> matA0;
                    Eigen::Matrix<double, 5, 1> matB0 =
                            -1 * Eigen::Matrix<double, 5, 1>::Ones();
                    if (pointSearchSqDis[4] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
                        for (int j = 0; j < 5; j++) {
                            matA0(j, 0) = frame_0_surf.points[pointSearchInd[j]].x;
                            matA0(j, 1) = frame_0_surf.points[pointSearchInd[j]].y;
                            matA0(j, 2) = frame_0_surf.points[pointSearchInd[j]].z;
                            // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
                            // 2));
                        }
                        // find the norm of plane
                        Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                        double negative_OA_dot_norm = 1 / norm.norm();
                        norm.normalize();

                        // Here n(pa, pb, pc) is unit norm of plane
                        bool planeValid = true;
                        for (int j = 0; j < 5; j++) {
                            // if OX * n > 0.2, then plane is not fit well
                            if (fabs(norm(0) *
                                     frame_0_surf.points[pointSearchInd[j]].x +
                                     norm(1) *
                                     frame_0_surf.points[pointSearchInd[j]].y +
                                     norm(2) *
                                     frame_0_surf.points[pointSearchInd[j]].z +
                                     negative_OA_dot_norm) > 0.02) {
                                planeValid = false;
                                break;
                            }
                        }

                        if (planeValid) {
                            Eigen::Vector3d last_point_a(
                                    frame_0_surf.points[pointSearchInd[0]].x,
                                    frame_0_surf.points[pointSearchInd[0]].y,
                                    frame_0_surf.points[pointSearchInd[0]].z);
                            Eigen::Vector3d last_point_b(
                                    frame_0_surf.points[pointSearchInd[2]].x,
                                    frame_0_surf.points[pointSearchInd[2]].y,
                                    frame_0_surf.points[pointSearchInd[2]].z);
                            Eigen::Vector3d last_point_c(
                                    frame_0_surf.points[pointSearchInd[4]].x,
                                    frame_0_surf.points[pointSearchInd[4]].y,
                                    frame_0_surf.points[pointSearchInd[4]].z);

                            Eigen::Vector3d center = (last_point_a + last_point_b + last_point_c) / 3.0;

                            FeatureCorPairs surfCor;
                            FeatureType featureType = FeatureType::Surf;
                            surfCor.setFeatureType(featureType);
                            surfCor.feature_ = frame_1.cloud_feature_.less_surf_feature_.points.at(l);
                            surfCor.corresponding_points_.push_back(frame_0_surf.points[pointSearchInd[0]]);
                            surfCor.corresponding_points_.push_back(frame_0_surf.points[pointSearchInd[2]]);
                            surfCor.corresponding_points_.push_back(frame_0_surf.points[pointSearchInd[4]]);
                            surfCor.dir_vec_ = norm;

                            surfCor.corresponding_center_ = PointT({static_cast<float>(center.x()),
                                                                    static_cast<float>(center.y()),
                                                                    static_cast<float>(center.z())});
                            surfCor.distance_ = 0;

                            newSurfCorGroup.push_back(surfCor);
                        }

                    }
                }
//            ROS_DEBUG_STREAM("frame " << i << ": surf " << newSurfCorGroup.size() << ", edge " << newEdgeCorGroup.size());
            }

            frame_in_window_[i].SetAssociationPairsGroup(newEdgeCorGroup, newSurfCorGroup);

        }

    }

    void LocalMapper::DataAssociationAdjacentGround() {

        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;

        for (int i = FIXED_SIZE; i < WINDOW_SIZE + 1; ++i) {

            Frame frame_0 = frame_in_window_[i-1];
            Frame frame_1 = frame_in_window_[i];

            Eigen::Matrix4d pose_0 = frame_0.state_.getTransformation();
            Eigen::Matrix4d pose_1 = frame_1.state_.getTransformation();

            Eigen::Matrix4d rel_T = extrinsic_parameters_.getTransform().inverse() * pose_0.inverse() * pose_1 * extrinsic_parameters_.getTransform();
            if (frame_0.cloud_feature_.ground_feature_.size() <= 20)
                continue;

            kdTree_surf_map_->setInputCloud(frame_0.cloud_feature_.ground_feature_.makeShared());

            FeatureCorPairsGroup newGroundCorGroup;

            int pointSearchSurfInd1, pointSearchSurfInd2, pointSearchSurfInd3;

            // find ground pairs
            PointCloudT frame_0_ground = frame_0.cloud_feature_.ground_feature_;
            PointCloudT::Ptr frame_1_ground(new PointCloudT);
            pcl::transformPointCloud(frame_1.cloud_feature_.ground_feature_, *frame_1_ground, rel_T);

            newGroundCorGroup.reserve(frame_1_ground->points.size());

            for (int l = 0; l < frame_1_ground->points.size(); ++l) {

                PointT pointSel = frame_1_ground->points.at(l);
                kdTree_surf_map_->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                bool planeValid = false;

                Eigen::Matrix<double, 5, 3> matA0;
                Eigen::Matrix<double, 5, 1> matB0 =
                        -1 * Eigen::Matrix<double, 5, 1>::Ones();
                if (pointSearchSqDis[4] < NEAREST_FEATURE_SEARCH_SQ_DIST) {
                    for (int j = 0; j < 5; j++) {
                        matA0(j, 0) = frame_0_ground.points[pointSearchInd[j]].x;
                        matA0(j, 1) = frame_0_ground.points[pointSearchInd[j]].y;
                        matA0(j, 2) = frame_0_ground.points[pointSearchInd[j]].z;
                        // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
                        // 2));
                    }
                    // find the norm of plane
                    Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
                    double negative_OA_dot_norm = 1 / norm.norm();
                    norm.normalize();

                    // Here n(pa, pb, pc) is unit norm of plane
                    bool planeValid = true;
                    for (int j = 0; j < 5; j++) {
                        // if OX * n > 0.2, then plane is not fit well
                        if (fabs(norm(0) *
                                 frame_0_ground.points[pointSearchInd[j]].x +
                                 norm(1) *
                                 frame_0_ground.points[pointSearchInd[j]].y +
                                 norm(2) *
                                 frame_0_ground.points[pointSearchInd[j]].z +
                                 negative_OA_dot_norm) > 0.01) {
                            planeValid = false;
                            break;
                        }
                    }

                    if (planeValid) {
                        Eigen::Vector3d last_point_a(
                                frame_0_ground.points[pointSearchInd[0]].x,
                                frame_0_ground.points[pointSearchInd[0]].y,
                                frame_0_ground.points[pointSearchInd[0]].z);
                        Eigen::Vector3d last_point_b(
                                frame_0_ground.points[pointSearchInd[2]].x,
                                frame_0_ground.points[pointSearchInd[2]].y,
                                frame_0_ground.points[pointSearchInd[2]].z);
                        Eigen::Vector3d last_point_c(
                                frame_0_ground.points[pointSearchInd[4]].x,
                                frame_0_ground.points[pointSearchInd[4]].y,
                                frame_0_ground.points[pointSearchInd[4]].z);

                        Eigen::Vector3d center = (last_point_a + last_point_b + last_point_c) / 3.0;

                        FeatureCorPairs surfCor;
                        FeatureType featureType = FeatureType::Ground;
                        surfCor.setFeatureType(featureType);
                        surfCor.feature_ = frame_1.cloud_feature_.ground_feature_.points.at(l);
                        surfCor.corresponding_points_.push_back(frame_0_ground.points[pointSearchInd[0]]);
                        surfCor.corresponding_points_.push_back(frame_0_ground.points[pointSearchInd[2]]);
                        surfCor.corresponding_points_.push_back(frame_0_ground.points[pointSearchInd[4]]);
                        surfCor.dir_vec_ = norm;

                        surfCor.corresponding_center_ = PointT({static_cast<float>(center.x()),
                                                                static_cast<float>(center.y()),
                                                                static_cast<float>(center.z())});
                        surfCor.distance_ = 0;

                        newGroundCorGroup.push_back(surfCor);
                    }

                }

            }
//            ROS_DEBUG_STREAM("frame " << i << ": ground " << newGroundCorGroup.size());

            frame_in_window_[i].SetAssociationGroundPairsGroup(newGroundCorGroup);
        }
    }

    void LocalMapper::PlaneAssociationAdjacent() {

        for (int i = FIXED_SIZE; i < WINDOW_SIZE + 1; ++i) {

            Frame frame_0 = frame_in_window_[i - 1];
            Frame frame_1 = frame_in_window_[i];

            Eigen::Matrix4d pose_0 = frame_0.state_.getTransformation();
            Eigen::Matrix4d pose_1 = frame_1.state_.getTransformation();

            Eigen::Matrix4d rel_T = extrinsic_parameters_.getTransform().inverse() * pose_0.inverse() * pose_1 * extrinsic_parameters_.getTransform();

            std::vector<PlaneFeature> frame0_plane = frame_0.cloud_feature_.plane_feature_;
            std::vector<PlaneFeature> frame1_plane = frame_1.cloud_feature_.plane_feature_;
//            std::cout << i << " frame plane size : " << frame1_plane.size()  << " - " << frame0_plane.size() << std::endl;

            PlaneCorPairsGroup planeCorPairsGroup;

            for (int j = 0; j < frame1_plane.size(); j++) {

                frame1_plane.at(j).Transform(rel_T);

                // 1- find nearest closest point and the center distance along the normal vector;
//                std::cout << j << ": ";
                std::vector<int> index_cp;
                for (int k = 0; k < frame0_plane.size(); ++k) {

                    float cp_angle = std::acos(frame1_plane.at(j).cp_.dot(frame0_plane.at(k).cp_) /
                                               (frame1_plane.at(j).cp_.norm() * frame0_plane.at(k).cp_.norm()))/M_PI *180.0;

                    Eigen::Vector3f center_v =  frame1_plane.at(j).center_ - frame0_plane.at(k).center_;
                    float center_distance_0 = std::abs(center_v.dot(frame0_plane.at(k).normal_));
                    float center_distance_1 = std::abs(center_v.dot(frame1_plane.at(j).normal_));
                    float center_distance_mean = (center_distance_0 + center_distance_1) / 2;

//                    std::cout << cp_angle << " " << center_distance_mean << " |";
                    if(cp_angle <= PLANE_ASSOCIATED_ANGLE || center_distance_mean <= PLANE_CLUSTER_CENTER_DISTANCE)
                        index_cp.push_back(k);
                }
//                std::cout << "| angle threshold: " << PLANE_ASSOCIATED_ANGLE  << " distance threshold: " << PLANE_CLUSTER_CENTER_DISTANCE << std::endl;

                // 2- find overlapping
                float max_overlapping_rate = 0;
                int max_index = 0;
//                std::cout << j << ": ";

                auto line_points_1 = frame1_plane.at(j).Get2DLine();
                Eigen::Vector2f point_1_0 = line_points_1.at(0);
                Eigen::Vector2f point_1_1 = line_points_1.at(0);

                for (int l = 0; l < index_cp.size(); ++l) {

                    auto line_points_0 = frame0_plane.at(index_cp.at(l)).Get2DLine();

                    Eigen::Vector2f point_0_0 = line_points_0.at(0);
                    Eigen::Vector2f point_0_1 = line_points_0.at(1);

                    float intersection_angle = 0;

                    if(point_0_0[0] <= point_1_0[0]){

                        if(point_1_0[0] <= point_0_1[0]){
                            //intersection
                            if(point_1_1[0] <= point_0_1[0])
                                intersection_angle = std::acos(point_1_1.dot(point_1_0)/(point_1_1.norm()*point_1_0.norm()));
                            else
                                intersection_angle = std::acos(point_1_0.dot(point_0_1)/(point_1_0.norm()*point_0_1.norm()));
                        }
                    }else {
                        if(point_1_1[0] >= point_0_0[0]){

                            if(point_1_1[0] >= point_0_1[0])
                                intersection_angle = std::acos(point_0_0.dot(point_0_1)/(point_0_0.norm()*point_0_1.norm()));
                            else
                                intersection_angle = std::acos(point_0_0.dot(point_1_1)/(point_0_0.norm()*point_1_1.norm()));
                        }
                    }

                    float angle_0 = std::acos(point_0_0.dot(point_0_1)/(point_0_0.norm()*point_0_1.norm()));
                    float angle_1 = std::acos(point_1_1.dot(point_1_0)/(point_1_1.norm()*point_1_0.norm()));

                    float rate_0 = intersection_angle/angle_0;
                    float rate_1 = intersection_angle/angle_1;

                    float rate = rate_0 >= rate_1 ? rate_0 : rate_1;

                    if(rate >= max_overlapping_rate){
                        max_overlapping_rate = rate;
                        max_index = index_cp.at(l);
                    }
                }
//                std::cout << std::endl;

                if(max_overlapping_rate >=  PLANE_ASSOCIATED_OVERLAPPING_RATE){
                    PlaneCorPair plane_pair(frame_0.cloud_feature_.plane_feature_.at(max_index).cp_,
                                            frame_1.cloud_feature_.plane_feature_.at(j).cp_, max_index, j);
                    planeCorPairsGroup.push_back(plane_pair);
                }

            }

            frame_in_window_[i].SetAssociationPlaneGroup(planeCorPairsGroup);

            if(SAVE_FEATURE_TMP_RESULT){

                std::string plane_2d_line_path = OUTPUT_PATH + std::to_string(i) + "_plane_2d_line_correspondence.txt";
                std::ofstream fout(plane_2d_line_path.c_str());

                PointCloudT cloud_0, cloud_1;
                for (int j = 0; j < frame0_plane.size(); ++j) {
                    cloud_0 += frame0_plane.at(j).cloud_;

                    auto line = frame0_plane.at(j).Get2DLine();
                    auto cp = frame0_plane.at(j).cp_;
                    fout << "0 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose() << std::endl;
                }

                for (int j = 0; j < frame1_plane.size(); ++j) {
                    cloud_1 += frame1_plane.at(j).cloud_;

                    auto line = frame1_plane.at(j).Get2DLine();
                    auto cp = frame1_plane.at(j).cp_;
                    fout << "1 " << line.at(0).transpose() << " " << line.at(1).transpose() << " " << cp.transpose() << std::endl;
                }

                fout.close();

                if (!cloud_0.empty()){
                    std::string cloud_save_file = OUTPUT_PATH + std::to_string(i) + "_correspondence_plane_cloud_0" + ".pcd";
                    pcl::io::savePCDFileBinary(cloud_save_file, cloud_0);
                }
                if (!cloud_1.empty()){
                    std::string cloud_save_file = OUTPUT_PATH + std::to_string(i) + "_correspondence_plane_cloud_1" + ".pcd";
                    pcl::io::savePCDFileBinary(cloud_save_file, cloud_1);
                }
            }

        }

    }

    MarginalizationFlag LocalMapper::CheckMarginalizationType(int &frame_count, bool is_moving)
    {

        // Similar to the vins-mono, we check whether the second latest frame is a keyframe.
        // If it is true, we keep it in the window, and marginalize the oldest frame;
        // Otherwise, we marginalize it and contain the pre-integration term.
        // Because of the characters of the LiDAR points, we use time or moving distance
        // or rotation angle as condition of the keyframes
        double time_diff = fabsf(frame_in_window_[frame_count - 2].state_.timestamp_ - frame_in_window_[frame_count - 1].state_.timestamp_);
//        ROS_DEBUG_STREAM("time_diff " << time_diff);

        Eigen::Matrix4d rel_T = frame_in_window_[frame_count - 2].state_.getTransformation().inverse() *
                                frame_in_window_[frame_count - 1].state_.getTransformation();

        Eigen::Vector3d rel_p = rel_T.block(0, 3, 3, 1);

        Eigen::Matrix3d rel_rot = rel_T.block(0, 0, 3, 3);
        Eigen::Vector3d rel_ypr = Utility::R2ypr(rel_rot);

        float moving_distance = rel_p.norm();

        bool c1 = (time_diff >= KEYFRAME_TIME_DIFF);
        bool c2 = (moving_distance >= KEYFRAME_DISTANCE);
        bool c3 = (rel_ypr[0] >= KEYFRAME_ROTATION || rel_ypr[1] >= KEYFRAME_ROTATION || rel_ypr[2] >= KEYFRAME_ROTATION);
        
        // std::cout << "moving_distance " << moving_distance << std::endl;


        if(moving_distance<KEYFRAME_STILL_DISTANCE)
        {
            is_still = 1;
        }
        else
        {
            is_still = 0;
        }

        is_turning = c3;
        
        if (is_moving)
        {
            if (c2 || c3)
                return MarginalizationFlag::MARGIN_OLD;
            else
                return MarginalizationFlag::MARGIN_SECOND_NEW;
        }
        else
        {
//            if (c1)
//                return MarginalizationFlag::MARGIN_OLD;
//            else
            return MarginalizationFlag::MARGIN_SECOND_NEW;
        }
    }

    void LocalMapper::Add2LocalMap(gp_lio::PointCloudT edge, gp_lio::PointCloudT surf,
                                   gp_lio::PointCloudT ground, std::vector<PlaneFeature> planes, State state) {

        edge_map_deque_.emplace_back(edge);
        surf_map_deque_.emplace_back(surf);
        ground_map_deque_.emplace_back(ground);
        plane_feature_deque_.emplace_back(planes);
        pose_deque_.emplace_back(state);

        int d_size = edge_map_deque_.size();
        if (d_size > WINDOW_SIZE) {
            edge_map_deque_.pop_front();
            surf_map_deque_.pop_front();
            ground_map_deque_.pop_front();
            plane_feature_deque_.pop_front();
            pose_deque_.pop_front();
        }
    }

    PointCloudT LocalMapper::GetLocalMap() {

        if (!local_map_->empty()) {
            return *local_map_;
        } else
            return PointCloudT();
    }

    void LocalMapper::AddClosedLocalMap(PointCloudT edge, PointCloudT surf, PointCloudT ground,
                                        std::vector <PlaneFeature> planes) {

        if(!closed_edge_map_->empty()){
            closed_edge_map_->clear();
            closed_surf_map_->clear();
            closed_ground_map_->clear();
            closed_plane_feature_.clear();
        }
        *closed_edge_map_ = edge;
        *closed_surf_map_ = surf;
        *closed_ground_map_ = ground;
        closed_plane_feature_ = planes;

    }

    void LocalMapper::AssembleLocalMap()
    {

        PointCloudT::Ptr edgeMap(new PointCloudT()), surfMap(new PointCloudT()), groundMap(new PointCloudT());

        edge_map_DS_->clear();
        surf_map_DS_->clear();
        ground_map_DS_->clear();

        for (int i = 0; i < FIXED_SIZE; ++i)
        {
            *edgeMap += frame_in_window_[i].GetTransformedEdgeCloud(extrinsic_parameters_);
            *surfMap += frame_in_window_[i].GetTransformedSurfCloud(extrinsic_parameters_);
            *groundMap += frame_in_window_[i].GetTransformedGroundCloud(extrinsic_parameters_);
        }

        if (!edge_map_deque_.empty() && !surf_map_deque_.empty() && !ground_map_deque_.empty()){
            int d_size = edge_map_deque_.size();
            for (int i = 0; i < d_size; ++i) {
                *edgeMap += TransformPointCloud(edge_map_deque_.at(i), pose_deque_.at(i));
                *surfMap += TransformPointCloud(surf_map_deque_.at(i), pose_deque_.at(i));
                *groundMap += TransformPointCloud(ground_map_deque_.at(i), pose_deque_.at(i));
            }
        }

        if(!closed_edge_map_->empty()){

//            edgeMap->clear();
//            surfMap->clear();
//            groundMap->clear();

            *edgeMap += *closed_edge_map_;
            *surfMap += *closed_surf_map_;
            *groundMap += *closed_ground_map_;
            ROS_ERROR("Add loop closed local map");

            closed_edge_map_->clear();
            closed_surf_map_->clear();
            closed_ground_map_->clear();
            closed_plane_feature_.clear();

        }

        downSize_filter_edge_map_.setInputCloud(edgeMap);
        downSize_filter_edge_map_.filter(*edge_map_DS_);

        downSize_filter_surf_map_.setInputCloud(surfMap);
        downSize_filter_surf_map_.filter(*surf_map_DS_);

        downSize_filter_ground_map_.setInputCloud(groundMap);
        downSize_filter_ground_map_.filter(*ground_map_DS_);

        kdTree_edge_map_->setInputCloud(edge_map_DS_);
        kdTree_surf_map_->setInputCloud(surf_map_DS_);
        kdTree_ground_map_->setInputCloud(ground_map_DS_);

        PointCloudT::Ptr local_map(new PointCloudT);
        *local_map = *surf_map_DS_ + *edge_map_DS_;
        *local_map = *local_map + *ground_map_DS_;
        if (!local_map_->empty()) local_map_->clear();
        *local_map_ = *local_map;

        if(SAVE_FEATURE_TMP_RESULT && !local_map_->empty()){
            std::string path = OUTPUT_PATH + "localMap.pcd";
            pcl::io::savePCDFileBinary(path, *local_map_);
        }
    }

    PointT LocalMapper::PointAssociatedToMap(PointT &pi, State &state)
    {

        Eigen::Vector3d pi_v(pi.x, pi.y, pi.z);
        auto pose = state.getTransformation() * extrinsic_parameters_.getTransform(); //{w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix3d rot = pose.block(0, 0, 3, 3);
        Eigen::Vector3d t = pose.block(0, 3, 3, 1);
        Eigen::Vector3d po_v = rot * pi_v + t;

        PointT po;
        po.x = po_v.x();
        po.y = po_v.y();
        po.z = po_v.z();
        po.intensity = pi.intensity;
        return po;
    }

    PointCloudT LocalMapper::TransformPointCloud(gp_lio::PointCloudT cloud_in, gp_lio::State state) {

        PointCloudT::Ptr transformedCloud(new PointCloudT());
        // {w}_{L_k}T = {w}_{B_k}T * {B}_{L}T
        Eigen::Matrix4d trans = state.getTransformation() * extrinsic_parameters_.getTransform();
        pcl::transformPointCloud(cloud_in, *transformedCloud, trans);
        return *transformedCloud;
    }

    void LocalMapper::SlideWindow(gp_lio::MarginalizationFlag marginalization_flag)
    {

        if (marginalization_flag == MARGIN_OLD)
        {
            Frame frame0 = frame_in_window_[0];
            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                frame_in_window_[i].swap(frame_in_window_[i + 1]);
            }
            frame_in_window_[WINDOW_SIZE].clear();
            frame_in_window_[WINDOW_SIZE] = frame_in_window_[WINDOW_SIZE - 1];
        }
        else
        {
            frame_in_window_[WINDOW_SIZE - 1].clear();
            frame_in_window_[WINDOW_SIZE - 1] = frame_in_window_[WINDOW_SIZE];
        }
    }
} // namespace gp_lio