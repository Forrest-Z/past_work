//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_VISUALIZATION_H
#define SRC_VISUALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

#include <vector>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

#include <gp_lio/utility/Utility.h>
#include <gp_lio/estimator/Parameters.h>

namespace gp_lio{
    class Visualization {
    public:

        Visualization();

        ~Visualization();

        void LoadConfig(ros::NodeHandle nh);

        void UpdateLocalWindow(std::vector<State> local_window_state,
                               std::vector<PointCloudT> local_window_feature,
                               std::vector<PointCloudT> local_window_ground,
                               std::vector<PointCloudT> local_window_plane);

        void UpdateLocalMap(PointCloudT local_map);

        void AddMarginOldCloud(State& state, PointCloudT& feature_cloud, PointCloudT& ground_cloud, PointCloudT& plane_cloud);

        void ShowCorrespondence(std::vector<std::pair<FeatureCorPairsGroup, State>> cors, ExternalParameters extrinsic_parameters);

        void Publish();

        void PublishRelocalization(std::vector<State> & local_window_states);

    private:

        ros::NodeHandle nh_;

        ros::Publisher local_feature_cloud_pub_;
        ros::Publisher local_ground_cloud_pub_;
        ros::Publisher local_plane_cloud_pub_;
        ros::Publisher local_traj_pub_;
        ros::Publisher local_map_pub_;

        ros::Publisher cors_pub_;

        ros::Publisher velocity_pub_;

        PointCloudT::Ptr local_feature_cloud_map_DS_;
        PointCloudT::Ptr local_ground_cloud_map_DS_;
        PointCloudT::Ptr local_plane_cloud_map_DS_;
        PointCloudT::Ptr local_trajctory_;
        PointCloudT::Ptr local_map_DS_;

        pcl::VoxelGrid<PointT> downsize_filter_;

        visualization_msgs::MarkerArray cors_msg_;

        visualization_msgs::MarkerArray velocity_msg_;

        int i_;

    };
}

#endif //SRC_VISUALIZATION_H
