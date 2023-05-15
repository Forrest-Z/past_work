/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:42:44
 * @LastEditors: luo
 * @LastEditTime: 2022-01-10 14:17:27
 */
#include "kalman_filter_pkg/graph_optimizer/graph_optimizer.hpp"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <glog/logging.h>

#include "kalman_filter_pkg/global_defination/global_defination.h"
#include "kalman_filter_pkg/tools/file_manager.hpp"

namespace KalmanFilter
{

GraphOptimizer::GraphOptimizer()
{
    InitWithConfig();
}

GraphOptimizer::~GraphOptimizer()
{


}

bool GraphOptimizer::InitWithConfig()
{
    std::string config_file_path = "/home/luo/work/filter_ws/src/kalman_filter_pkg/config/graph_optimizer.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout << "-----------------Init LIO Localization, Backend-------------------" << std::endl;

    // a. estimation output path:
    InitDataPath(config_node);
    // b. key frame selection config:
    InitKeyFrameSelection(config_node);
    // c. sliding window config:
    InitSlidingWindow(config_node);
    // d. IMU pre-integration:
    InitIMUPreIntegrator(config_node);

    return true;

}

bool GraphOptimizer::InitDataPath(const YAML::Node& config_node)
{
    std::string data_path = config_node["data_path"].as<std::string>();
    if (data_path == "./") {
        data_path = "/home/luo/work/filter_ws/src/kalman_filter_pkg";
    }

    if (!FileManager::CreateDirectory(data_path + "/slam_data"))
        return false;

    trajectory_path_ = data_path + "/slam_data/trajectory";
    if (!FileManager::InitDirectory(trajectory_path_, "Estimated Trajectory"))
        return false;

    return true;

}

bool GraphOptimizer::InitKeyFrameSelection(const YAML::Node& config_node)
{
    key_frame_config_.max_distance = config_node["key_frame"]["max_distance"].as<float>();
    key_frame_config_.max_interval = config_node["key_frame"]["max_interval"].as<float>();

    return true;

}

bool GraphOptimizer::InitSlidingWindow(const YAML::Node& config_node)
{
    // init sliding window:
    const int sliding_window_size = config_node["sliding_window_size"].as<int>();
    ceres_sliding_window_ptr_ = std::make_shared<CeresGraphOptimizer>(sliding_window_size);

    // select measurements:
    measurement_config_.source.map_matching = config_node["measurements"]["map_matching"].as<bool>();
    measurement_config_.source.imu_pre_intrgration = config_node["measurements"]["imu_pre_integration"].as<bool>();

    // get measurement noises, pose:
    measurement_config_.noise.lidar_odomtry.resize(6);
    measurement_config_.noise.map_matching.resize(6);
    for (int i = 0; i < 6; ++i) {
        measurement_config_.noise.lidar_odomtry(i) =
            config_node["lidar_odometry"]["noise"][i].as<double>();
        measurement_config_.noise.map_matching(i) =
            config_node["map_matching"]["noise"][i].as<double>();
    }

    // get measurement noises, position:
    measurement_config_.noise.gnss_position.resize(3);
    for (int i = 0; i < 3; i++) {
        measurement_config_.noise.gnss_position(i) =
            config_node["gnss_position"]["noise"][i].as<double>();
    }

    return true;

}

bool GraphOptimizer::InitIMUPreIntegrator(const YAML::Node& config_node)
{
    imu_pre_integrator_ptr_ = nullptr;
    
    if (measurement_config_.source.imu_pre_intrgration) {
        imu_pre_integrator_ptr_ = std::make_shared<IMUPreIntegrator>(config_node["imu_pre_integration"]);
    }

    return true;
}


bool GraphOptimizer::UpdateIMUPreIntrgration(const IMUData& imu_data)
{
    if ( !measurement_config_.source.imu_pre_intrgration || nullptr == imu_pre_integrator_ptr_ )
        return false;
    
    if (
        !imu_pre_integrator_ptr_->IsInited() ||
        imu_pre_integrator_ptr_->Update(imu_data)
    ) {
        return true;
    }

    return false;
    
}

void GraphOptimizer::ResetParam()
{
    has_new_key_frame_ = false;
    has_new_optimized_ = false;
}

bool GraphOptimizer::MaybeNewKeyFrame(const PoseData& laser_odom, const IMUData& imu_data)
{
    static KeyFrame last_key_frame;

    if(key_frames_.lidar.empty())
    {
        if(imu_pre_integrator_ptr_)
        {
            imu_pre_integrator_ptr_->Init(imu_data);
        }

        has_new_key_frame_ = true;
    }else if((laser_odom.pose.block<3, 1>(0, 3) - last_key_frame.pose.block<3, 1>(0, 3)).lpNorm<1>() 
                        > key_frame_config_.max_distance || 
                        (laser_odom.time - last_key_frame.time) > key_frame_config_.max_interval)
    {
        if(imu_pre_integrator_ptr_)
        {
            imu_pre_integrator_ptr_->Reset(imu_data, imu_pre_intrgration_);

            std::cout << "GraphOptimizer::MaybeNewKeyFrame----Reset-----P = "<<std::endl << imu_pre_intrgration_.P << std::endl;
        }

        has_new_key_frame_ = true;
    }else 
    {
        has_new_key_frame_ = false;
    }

    if(has_new_key_frame_)
    {
        // create key frame for lidar odometry, relative pose measurement:
        current_key_frame_.time = laser_odom.time;
        current_key_frame_.index = key_frames_.lidar.size();
        current_key_frame_.pose = laser_odom.pose;
        current_key_frame_.vel.v = laser_odom.Vel.v;
        current_key_frame_.vel.w = laser_odom.Vel.w;

        key_frames_.lidar.push_back(current_key_frame_);

        last_key_frame = current_key_frame_;
    }

    return has_new_key_frame_;

}

bool GraphOptimizer::Update()
{
    static KeyFrame last_key_frame_ = current_key_frame_;

    std::cout << " GraphOptimizer Update 00" << std::endl;
    
    if(ceres_sliding_window_ptr_->GetNumParamBlocks() == 0)
    {
        ceres_sliding_window_ptr_->AddPRVAGParam(current_key_frame_, true);
    }else 
    {
        ceres_sliding_window_ptr_->AddPRVAGParam(current_key_frame_, false);
    }

    //get num of vertices
    const int N = ceres_sliding_window_ptr_->GetNumParamBlocks();

    //get param block ID, 
    const int param_index_j = N - 1;

    //add unary constraints
    std::cout << " GraphOptimizer Update 44" << std::endl;
    //a. map matching 
    // if(N > 0 && measurement_config_.source.map_matching)
    // {
    //     //get prior position measurement
    //     Eigen::Matrix4d prior_pose = current_map_matching_pose_.pose.cast<double>();
    //     std::cout << " GraphOptimizer Update 55" << std::endl;
    //     //add constraint
    //     ceres_sliding_window_ptr_->AddPRVAGMapMatchingPoseFactor(param_index_j, prior_pose,
    //                                     measurement_config_.noise.map_matching);
    // }
    // std::cout << " GraphOptimizer Update 66" << std::endl;

    //add binary  constraints
    if(N > 1)
    {
        //get param block ID 
        const int param_index_i = N -2;

        //a. lidar frontend
        //get relative pose measurement
        Eigen::Matrix4d relative_pose = (last_key_frame_.pose.inverse() * current_key_frame_.pose).cast<double>();
        //add constraint. lidar frontend
            std::cout << " GraphOptimizer Update 77" << std::endl;
        ceres_sliding_window_ptr_->AddPRVAGRelativePoseFactor(
            param_index_i, param_index_j, relative_pose, measurement_config_.noise.lidar_odomtry
        );
        std::cout << " GraphOptimizer Update 88" << std::endl;

        std::cout << " GraphOptimizer imu_pre_intrgration_" << std::endl << imu_pre_intrgration_.P << std::endl;
        //b. IMU pre-integration
        if(measurement_config_.source.imu_pre_intrgration)
        {
            ceres_sliding_window_ptr_->AddPRVAGIMUPreIntegrationFactor(
                param_index_i, param_index_j, imu_pre_intrgration_
            );
        }
    }
    std::cout << " GraphOptimizer Update 99" << std::endl;
    //move forward
    last_key_frame_ = current_key_frame_;

    return true;

}

bool GraphOptimizer::MaybeOptimized()
{
    if(ceres_sliding_window_ptr_->Optimize())
    {
        has_new_optimized_ = true;

        return true;
    }

    return false;

}

bool GraphOptimizer::Update(const PoseData& lidar_odom, const IMUData& imu_data)
{
    ResetParam();

    if(MaybeNewKeyFrame(lidar_odom, imu_data))
    {
        Update();
        MaybeOptimized();
    }

    return true;
}


bool GraphOptimizer::HasNewKeyFrame()
{
    return has_new_key_frame_;

}
bool GraphOptimizer::HasNewOptimized()
{
    return has_new_optimized_;
}

void GraphOptimizer::GetLatestKeyFrame(KeyFrame& key_frame)
{
    key_frame = current_key_frame_;

}
void GraphOptimizer::GetLatestOptimizedOdometry(KeyFrame& key_frame)
{
    ceres_sliding_window_ptr_->GetLatestOptimizedKeyFrame(key_frame);

}
void GraphOptimizer::GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque)
{
    key_frames_deque.clear();

    ceres_sliding_window_ptr_->GetOptimizedKeyFrames(key_frames_.optimized);

    key_frames_deque.insert(key_frames_deque.end(),
                            key_frames_.optimized.begin(), key_frames_.optimized.end()
                            );

}

}