/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:43:45
 * @LastEditors: luo
 * @LastEditTime: 2021-12-24 11:59:34
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>
#include <mutex>
#include <yaml-cpp/yaml.h>

#include "kalman_filter_pkg/sensor_data/pose_data.hpp"
#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/sensor_data/key_frame.hpp"

#include "kalman_filter_pkg/imu_pre_integrator/imu_pre_integrator.hpp"
#include "kalman_filter_pkg/graph_optimizer/ceres_graph_optimizer.hpp"

namespace KalmanFilter
{
class GraphOptimizer
{
public:
    GraphOptimizer();
    ~GraphOptimizer();

    bool Update(const PoseData& lidar_odom, const IMUData& imu_data);
    bool UpdateIMUPreIntrgration(const IMUData& imu_data);

    bool HasNewKeyFrame();
    bool HasNewOptimized();

    void GetLatestKeyFrame(KeyFrame& key_frame);
    void GetLatestOptimizedOdometry(KeyFrame& key_frame);
    void GetOptimizedKeyFrames(std::deque<KeyFrame>& key_frames_deque);

private:
    bool InitWithConfig();

    bool InitDataPath(const YAML::Node& config_node);
    bool InitKeyFrameSelection(const YAML::Node& config_node);
    bool InitSlidingWindow(const YAML::Node& config_node);
    bool InitIMUPreIntegrator(const YAML::Node& config_node);

    void ResetParam();
    bool MaybeNewKeyFrame(const PoseData& laser_odom, const IMUData& imu_data);


    bool Update();
    bool MaybeOptimized();


    
private:
  std::string trajectory_path_ = "";

  bool has_new_key_frame_ = false;
  bool has_new_optimized_ = false;

  KeyFrame current_key_frame_;
  PoseData current_map_matching_pose_;
  

  struct {
    std::deque<KeyFrame> lidar;
    std::deque<KeyFrame> optimized;
    std::deque<KeyFrame> reference;
  } key_frames_;

  std::shared_ptr<IMUPreIntegrator> imu_pre_integrator_ptr_;
  IMUPreIntegrator::IMUPreIntrgration imu_pre_intrgration_;

  // key frame config:
  struct {
    float max_distance;
    float max_interval;
  } key_frame_config_;

  std::shared_ptr<CeresGraphOptimizer> ceres_sliding_window_ptr_;

  //measurement config
  struct MeasurementConfig {
    struct {
      bool map_matching = false;
      bool imu_pre_intrgration = false;
    } source;

    struct {
      Eigen::VectorXd lidar_odomtry;
      Eigen::VectorXd map_matching;
      Eigen::VectorXd gnss_position;
    } noise;
  };

  MeasurementConfig measurement_config_;

};
    
} // namespace KalmanFilter
