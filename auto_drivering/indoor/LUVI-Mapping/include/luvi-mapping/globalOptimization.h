//
// Created by xc on 2021/3/8.
//

#ifndef ALOAM_VELODYNE_GLOBALOPTIMIZATION_H
#define ALOAM_VELODYNE_GLOBALOPTIMIZATION_H


// ros
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "pcl_conversions/pcl_conversions.h"
#include "nlink_parser/LinktrackNode2.h"
#include "nlink_parser/LinktrackNodeframe2.h"
#include "zed_interfaces/PoseStampedArray.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point.h>


// std
#include <sys/file.h>
#include <boost/filesystem.hpp>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#include <queue>
#include <map>
#include <array>
#include <string>
//pcl
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/sam/RangeFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/sam/BearingRangeFactor.h>

#include "keyFrame.h"
#include "luvi-mapping/utility/file_manger.h"


using  namespace gtsam;
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::A; // pose3 (x,y,z,r,p,y)


namespace LIRO{
    class GlobalOptimization{
    public:
        GlobalOptimization();
        ~GlobalOptimization();
        void LoopClosureThread();
        void Run();

    private:
        void LoadMap();
        void ArucoCallback(const zed_interfaces::PoseStampedArrayConstPtr &aruco_msg);
        void UwbCallback(const nlink_parser::LinktrackNodeframe2ConstPtr& uwb);
        void LidarOdomCallback(const nav_msgs::OdometryConstPtr& odom);
        void ImuCallback(const sensor_msgs::ImuConstPtr& imu);
        void SurfPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& surf);
        void CornerPointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& corner);
        void Command();
        bool SaveMap();
        void SaveUwbData();
        void SaveArucoData();
        void UwbDataPrepare();
        void ArucoDataPrepare();
        void GauessArucoInitial();
        void ArucoVisualization();
        bool AddKeyFrame(KeyFrame& keyframe);
        void GlobalMapPublish();
        void GlobalPathPublish();
        bool DetectLoopClosure(int& latest_id,int& closed_id);
        void PerformLoopClosure();
        void CorrectGlobalKeyFramePose();
        void CorrectLIOPath();
        void AddLoopAndRangFactor();
        void AddLIFactor();
        void ArucoOptimization();
        void RestLIOOptimization();
        void AllocateMemory();
        gtsam::Pose3 MaritxToGtsamPose(const Eigen::Matrix4d& tans);

    public:

    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_surf_pointcloud_;
        ros::Subscriber sub_corner_pointcloud_;
        ros::Subscriber sub_lidar_odom_;
        ros::Subscriber sub_imu_;
        ros::Subscriber sub_uwb_;
        ros::Subscriber sub_aruco_;
        ros::Publisher pub_global_map_;
        ros::Publisher pub_corner_map_;
        ros::Publisher pub_surf_map_;
        ros::Publisher pub_global_path_lo_;
        ros::Publisher pub_global_path_lio_;
        ros::Publisher pub_global_path_liro_;
        ros::Publisher pub_uwb_data_flag_;
        ros::Publisher pub_aruco_;
        ros::Publisher pub_aruco_opt_;

        // gtsam
        gtsam::Pose3 lidar2imu_,imu2lidar_;
        NonlinearFactorGraph gtSAMgraph_li_,gtSAMgraph_odom_,gtSAMgraph_aruco_;
        Values initialEstimate_li_,initialEstimate_odom_, initialEstimate_aruco_;
        bool lio_opt_initializd_;
        double isam_relinearize_threshold_;
        ISAM2 *isam_li_,*isam_odom_, *isam_aruco_;
        Values isamCurrentEstimate_li_,isamCurrentEstimate_odom_, isamCurrentEstimate_aruco_;
        gtsam::PreintegratedImuMeasurements *imu_integrator_;
        gtsam::Pose3 prev_pose_;
        gtsam::Vector3 prev_vel_;
        gtsam::NavState prev_state_;
        gtsam::imuBias::ConstantBias prev_bias_;
        float imu_acc_noise_,imu_gyr_noise_,
        imu_acc_bias_noise_,imu_gyr_bias_noise_,imu_gravity_;
        gtsam::noiseModel::Diagonal::shared_ptr prior_pose_noise_,prior_vel_noise_,prior_bias_noise_,
        correction_noise1_;
        double odom_t_noise_,odom_r_noise_;
        gtsam::Vector noise_model_between_bias_;
        double last_imu_opt_t_;

        std::queue<zed_interfaces::PoseStampedArrayConstPtr> aruco_queue_;
        std::queue<nlink_parser::LinktrackNodeframe2ConstPtr> uwb_queue_;
        std::queue<sensor_msgs::PointCloud2ConstPtr> surf_cloud_queue_;
        std::queue<sensor_msgs::PointCloud2ConstPtr> corner_cloud_queue_;
        std::queue<sensor_msgs::ImuConstPtr> imu_queue_;
        std::queue<nav_msgs::OdometryConstPtr> lidar_odom_queue_;
        pcl::VoxelGrid<pcl::PointXYZI> surf_filter_for_visualization_;
        pcl::VoxelGrid<pcl::PointXYZI> corner_filter_for_visualization_;
        pcl::VoxelGrid<pcl::PointXYZI> surf_filter_for_icp_;
        pcl::VoxelGrid<pcl::PointXYZI> corner_filter_for_icp_;

        std::vector<LIRO::KeyFrame> global_kf_v_;
        double odom_timestamp_;
        float keyframe_time_diff_;
        float keyframe_distance_;
        float keyframe_rotation_;

        // loop closure
        bool loop_is_closed_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_key_pose3d_;
        pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_history_key_poses_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr latest_keyframe_cloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr near_history_keyframe_cloud_;
        pcl::IterativeClosestPoint<pcl::PointXYZI,pcl::PointXYZI> icp_;
        Eigen::Matrix4d latest_keyframe_pose_,closest_keyframe_pose_;
        double icp_fitness_score_;
        unsigned int matching_min_id_distance;
        int loop_count;
        double coef_loop_closure_;
        double loop_closure_frequence_;
        double loop_closure_search_radius_;
        int history_keyframe_num_;
        pcl::PointCloud<pcl::PointXYZI> map_cloud_,global_path_lio_,global_path_lo_,global_path_liro_;
        Eigen::Matrix4d extrinsic_maritx4_;
        std::mutex mutx_;
        std::mutex kf_mutx_;

        std::string data_dir_path_;
        std::map<int,std::map<int,double>> uwb_anchor_map_;
        FileManager file_manager_;
        std::map<int,std::vector<std::array<double,14>>> aruco_map_;

        pcl::PointCloud<pcl::PointXYZI> global_map_;
        gtsam::Pose3 cam2lidar_;
        std::map<int,std::array<double,7>> aruco_initial_;
        std::map<int,std::array<double,7>> aruco_optimized_;
        std::vector<gtsam::Pose3> debug_data;
        std::vector<int> aruco_id_vec;

    };
}
#endif //ALOAM_VELODYNE_GLOBALOPTIMIZATION_H
