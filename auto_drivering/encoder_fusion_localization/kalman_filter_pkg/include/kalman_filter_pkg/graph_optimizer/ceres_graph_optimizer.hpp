/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:43:34
 * @LastEditors: luo
 * @LastEditTime: 2021-12-24 18:56:17
 */
#pragma once

#include <memory>

#include <string>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <deque>

#include <Eigen/Eigen>
#include <Eigen/Core>
#include <Eigen/Dense>


#include "kalman_filter_pkg/sensor_data/key_frame.hpp"
#include "kalman_filter_pkg/imu_pre_integrator/imu_pre_integrator.hpp"
#include <ceres/ceres.h>

#include "kalman_filter_pkg/graph_optimizer/params/param_pravg.hpp"
#include "kalman_filter_pkg/graph_optimizer/factors/factor_prvag_imu_pre_intrgration.hpp"
#include "kalman_filter_pkg/graph_optimizer/factors/factor_prvag_map_matching_pose.hpp"
#include "kalman_filter_pkg/graph_optimizer/factors/factor_prvag_marginalization.hpp"
#include "kalman_filter_pkg/graph_optimizer/factors/factor_prvag_relative_pose.hpp"

namespace KalmanFilter
{
class CeresGraphOptimizer
{
public:
    static const int INDEX_P = 0;
    static const int INDEX_R = 3;
    static const int INDEX_V = 6;
    static const int INDEX_A = 9;
    static const int INDEX_G = 12;

    struct OptimizedKeyFrame
    {
        double time;
        double prvag[15];
        double fixed = false;
    };

    struct ResidualMapMatchingPose
    {
        int param_index;

        Eigen::VectorXd m;
        Eigen::MatrixXd I;  
    };

    struct ResidualRelativePose
    {
        int param_index_i;
        int param_index_j;

        Eigen::VectorXd m;
        Eigen::MatrixXd I;
    };

    struct ResidualIMUPreIntegration
    {
        int param_index_i;
        int param_index_j;

        double T;
        Eigen::Vector3d g;
        Eigen::VectorXd m;
        Eigen::MatrixXd I;
        Eigen::MatrixXd J;

    };


    CeresGraphOptimizer(const int N);
    ~CeresGraphOptimizer();

    //add parameter block for LIO key frame
    void AddPRVAGParam(const KeyFrame &lio_key_frame, const bool fixed);

    //add residual block for relatice pose constraint from lidar frontend
    void AddPRVAGRelativePoseFactor(const int param_index_i, const int param_index_j,
                    const Eigen::Matrix4d& relative, const Eigen::VectorXd& noise);

    //add residual block for prior pose constraint from map matching
    void AddPRVAGMapMatchingPoseFactor(const int param_index, const Eigen::Matrix4d& prior_pose, 
                                        const Eigen::VectorXd& noise);

    //add residual block for IMU pre-intrgration constraint from IMU mewsurement
    void AddPRVAGIMUPreIntegrationFactor(const int param_index_i, const int param_index_j,
                                            const IMUPreIntegrator::IMUPreIntrgration &imu_pre_intrgration);


    // get num of paramter blocks
    int GetNumParamBlocks();

    //do optimization
    bool Optimize();

    //creat information matrix from measurement noise specification
    Eigen::MatrixXd GetInformationMatrix(const Eigen::VectorXd& noise);


    bool GetLatestOptimizedKeyFrame(KeyFrame& optimized_key_frame);
    bool GetOptimizedKeyFrames(std::deque<KeyFrame> &optimized_key_frames);

private:
    const int kWindowSize;

    struct {
        //a. loss function
        std::unique_ptr<ceres::LossFunction> loss_function_ptr;

        //b. solver
        ceres::Solver::Options options;
    } config_;

    //c data buffer
    //c.1 param blocks
    std::vector<OptimizedKeyFrame> optimized_key_frames_;

    //c.2 residual blocks
    FactorPRVAGMapMatchingPose *GetResMapMatchingPose(const ResidualMapMatchingPose &res_map_matching_pose);
    FactorPRVAGRelativePose *GetResRelativePose(const ResidualRelativePose& res_relative_pose);
    FactorPRVAGIMUPreIntrgration *GetResIMUPreIntegration(const ResidualIMUPreIntegration& res_imu_pre_integration);

    struct {
        std::deque<ResidualMapMatchingPose> map_matchin_pose;
        std::deque<ResidualRelativePose> relative_pose;
        std::deque<ResidualIMUPreIntegration> imu_pre_integration;
    } residual_blocks_;
};
    
} // namespace KalmanFilter

