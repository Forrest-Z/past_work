//
// Created by wchen on 2019/12/5.
//

#ifndef SRC_ESTIMATOR_H
#define SRC_ESTIMATOR_H

// ros
#include <ros/ros.h>

// STL
#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <math.h>

// ceres
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <ceres/autodiff_cost_function.h>
#include <ceres/numeric_diff_cost_function.h>

// gp_lio
#include "gp_lio/utility/Utility.h"
#include "gp_lio/utility/tic_toc.h"
#include "gp_lio/feature/ImuPreintegration.h"
#include "gp_lio/estimator/Parameters.h"
#include "gp_lio/mapper/LocalMapper.h"
#include "gp_lio/mapper/GlobalMapper.h"
#include "gp_lio/feature/LidarFeature.h"
#include "gp_lio/factor/PoseLocalParameterization.h"
#include "gp_lio/factor/ImuFactor.h"
#include "gp_lio/factor/LidarFactor.h"
#include "gp_lio/factor/MarginalizationFactor.h"
#include "gp_lio/initial/StaticInitializer.h"
#include "gp_lio/visualization/Visualization.h"

namespace gp_lio
{

class Estimator
{

public:
    Estimator();

    ~Estimator();

    void LoadConfig(ros::NodeHandle &nh);

    void ProcessMeasurements(MeasurementsGroupType &measurements_group);

    State GetUpdatedState();

    void LoadPosegraph();

    void SavePosegraph();

    LidarFeature lidar_feature_extractor_;

private:
    void ClearVariables();

    void ProcessImuMeasurement(ImuMeasurement &imu_measurement);

    // timestamps assignment and feature extraction
    void ProcessCloudMeasurement(CloudMeasurement &cloud_measurement, PointCloudT &cloud_last, Eigen::Matrix4d &transform_matrix);

    Eigen::Matrix4d getPose(CloudMeasurement &cloud_measurement);

    void DistortionPreCompensation(CloudFeature &lidar_feature);

    void FeaturesTransform(PointCloudT &feature, double &cloud_time);

    bool Init();

    void SolveOdometry();

    void Optimization();

    void States2Para();

    void Para2States();

    void SlideWindow();

    void Publish();

    void UpdateFromPoseGraph();

    void UpdateFromRelocalization(State & updated_state);

    void PublishRelocalization();

private:

    Parameters parameters_;
    LocalMapper localMapper_;
    GlobalMapper globalMapper_;
    StaticInitializer static_initializer_;
    Visualization visualizer_;
    // LidarFeature lidar_feature_extractor_;

    int frame_count_;

    Eigen::Vector3d gravity_;

    // variables in window
    ImuPreintegration *imu_preintegration_array_[(WINDOW_SIZE + 1)]; // nullptr

    PointCloudT cloud_last;
    Eigen::Matrix4d pose_t_last;

    // variables using in processing imu
    State initial_state_;
    bool first_pc_;
    bool first_imu_;
    float last_timestamp_;
    Eigen::Vector3d last_acc_;
    Eigen::Vector3d last_gyr_;
    int imu_propagation_times_;
    std::vector<double> dt_buf_[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> linear_acceleration_buf_[(WINDOW_SIZE + 1)];
    std::vector<Eigen::Vector3d> angular_velocity_buf_[(WINDOW_SIZE + 1)];
    bool is_moving_;
    int moving_kf_count_;

    // variables using in_ LidarFeature pre-compensation
    std::vector<State> temp_states_high_freq_;
    State last_state_;

    // variables using in processing
    SolverFlag solver_flag_;
    // para used in ceres
    double para_pose_[WINDOW_SIZE + 1][SIZE_POSE];
    double para_velocity_bias_[WINDOW_SIZE + 1][SIZE_VELOCITY_BIAS];
    double para_ex_pose_[1][SIZE_POSE];

    State latest_optimized_state_;

    // marginalization
    MarginalizationFlag marginalization_flag_;
    MarginalizationInfo *last_marginalization_info_;
    std::vector<double *> last_marginalization_parameter_blocks_;

    // mapper
    std::thread global_mapper_thread_;
    std::thread global_mapper_pub_thread_;

    // global keyframes are shared between odometry estimator and  global mapper.
    std::condition_variable kfs_con_;
    std::mutex kfs_mutex_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace gp_lio

#endif //SRC_ESTIMATOR_H
