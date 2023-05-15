/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-31 10:53:29
 * @LastEditors: luo
 * @LastEditTime: 2022-01-14 14:56:59
 */
//
// Created by ss on 2021-07-05.
//

#ifndef LOCALIZATION_IMUPREINTEGRATION_H
#define LOCALIZATION_IMUPREINTEGRATION_H
//local
#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"
// #include "file_manager.h"
// #include "parameters.h"
//ros
#include "ros/ros.h"
#include "ros/subscriber.h"
#include "ros/publisher.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

//std
#include <iostream>
#include <mutex>
#include <vector>
#include <queue>
#include <map>
#include <array>
#include <string>
#include <thread>
//pcl

//gtsam
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

using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)


namespace KalmanFilter{

    class IMUOdompreintegration{
    public:
        IMUOdompreintegration(ros::NodeHandle& nh);
        ~IMUOdompreintegration();

        void run();

    private:
        void ImuCallBack(const sensor_msgs::ImuConstPtr &imu_raw);
        void PriroOdometeyCallBack(const nav_msgs::OdometryConstPtr &odomMsg);
        void PriroEncoderOdomCallBack(const nav_msgs::OdometryConstPtr &encoder_msg);
        
        bool HasData();
        bool ValidData();
        bool UpdatePreintegration();
        bool PublishData();
        bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur);

        void resetOptimization();
        void resetParams();

        void Command();


    public:

    private:
        // FileManager file_manager1_;

        ros::NodeHandle nh_;
        ros::Subscriber subImu_;
        ros::Subscriber subPriorOdometry_;
        ros::Subscriber encoder_sub_;

        ros::Publisher pubImuOdometry_;

        std::shared_ptr<OdometryPublisher> pub_li_odom_ptr_;

        std::mutex mutex_;

        std::deque<sensor_msgs::ImuConstPtr> imu_opt_buffer_;
        std::deque<sensor_msgs::ImuConstPtr> imu_odom_buffer_;
        std::deque<nav_msgs::OdometryConstPtr> prior_odom_buffer_;
        std::deque<nav_msgs::OdometryConstPtr> prior_encoder_odom_buffer_;

        nav_msgs::OdometryConstPtr current_prior_odom_pose_;
        nav_msgs::OdometryConstPtr cur_encoder_prior_odom_pose_;
        gtsam::Pose3 last_encoder_prior_odom_pose_;
        double curPriorOdomTime_;

        bool lio_opt_initialized_;
        Eigen::Matrix4f li_odometry_ = Eigen::Matrix4f::Identity();
        Eigen::Vector3f li_velocity_ = Eigen::Vector3f::Zero();

        //gtsam
        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise_; // 先验位置噪声
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise_;  // 先验速度噪声
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise_; // 先验偏置噪声

        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise_;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2_;

        gtsam::noiseModel::Diagonal::shared_ptr encoder_correctionNoise_;
        gtsam::noiseModel::Diagonal::shared_ptr encoder_correctionNoise2_;
        
        gtsam::Vector noiseModelBetweenBias_;

        gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
        gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

        std::deque<sensor_msgs::Imu> imuQueOpt_;
        std::deque<sensor_msgs::Imu> imuQueImu_;

        gtsam::Pose3 prevPose_;  // 上一时刻估计imu的位姿信息
        gtsam::Vector3 prevVel_; // 上一时刻估计imu的速度信息
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;

        gtsam::NavState prevStateOdom_;
        gtsam::imuBias::ConstantBias prevBiasOdom_;

        // IMU
        float imuAccNoise_;
        float imuGyrNoise_;
        float imuAccBiasN_;
        float imuGyrBiasN_;
        float imuGravity_;


        bool doneFirstOpt_ = false;
        double lastImuTime_imu_ = -1;
        double lastImuTime_opt_ = -1;

        gtsam::ISAM2 isam_li_;
        gtsam::NonlinearFactorGraph graphFactors_li_;
        gtsam::Values initialEstimateValues_li_;
        gtsam::Values isamCurrentEstimate_li_;   //get result to publish

        const double delta_t_ = 0;

        int key = 1;

        Eigen::Matrix4d extrinsic_maritx4_;
        gtsam::Pose3 imu2Lidar_;
        gtsam::Pose3 lidar2Imu_;

        Eigen::Matrix4d encoder_extrinsic_maritx_;
        gtsam::Pose3 encoder2lidar_;
        gtsam::Pose3 lidar2encoder_;

        std::vector<std::pair<double,std::array<float ,7>>> odom_li_;


    };
}



#endif //LOCALIZATION_IMUPREINTEGRATION_H
