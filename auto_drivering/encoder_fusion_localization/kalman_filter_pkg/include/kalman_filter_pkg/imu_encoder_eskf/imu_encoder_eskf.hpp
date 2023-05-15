/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-26 14:52:37
 * @LastEditors: luo
 * @LastEditTime: 2021-11-29 10:13:50
 */
#pragma once

#include <iostream>
#include <ros/ros.h>
#include <deque>

#include "kalman_filter_pkg/sensor_data/imu_data.hpp"
#include "kalman_filter_pkg/subscriber/imu_subscriber.hpp"

#include "kalman_filter_pkg/sensor_data/encoder_data.hpp"
#include "kalman_filter_pkg/subscriber/encoder_subsciber.hpp"

#include "kalman_filter_pkg/publisher/odometry_publisher.hpp"

#include "kalman_filter_pkg/sensor_data/state_data.hpp"

#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace KalmanFilter
{
class ImuEncoderESKF
{
public:
    ImuEncoderESKF(ros::NodeHandle& nh);
    ~ImuEncoderESKF();

    bool Run();
    bool Calibration();
    bool ReadData();
    bool HasData();
    bool ValidData();
    bool SetInitPose();
    
    bool CalculateEncoderPosition(EncoderData& encoder_msg, IMUData& imu_msg, double dt);

    bool Filter();
    bool Predict();
    bool Correct();

    bool PublishData();

    Eigen::Matrix3d GetSkewMatrix(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d w;
        w << 0., -v(2), v(1), 
             v(2), 0., -v(0), 
             -v(1), v(0), 0.;

        return w;
    }


private:
    ros::NodeHandle nh_;

    std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
    std::deque<IMUData> imu_data_buff_;

    std::shared_ptr<EncoderSubscriber> encoder_sub_ptr_;
    std::deque<EncoderData> encoder_data_buff_;

    std::shared_ptr<OdometryPublisher> fusion_odom_pub_ptr_;
    
    IMUData last_imu_data_;
    IMUData current_imu_data_;
    EncoderData current_encoder_data_;


    Eigen::Matrix4f encoder_to_imu_ = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f init_pose_ = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f fusion_pose_ = Eigen::Matrix4f::Identity();

    State state_;
    ErrorState error_state_;

    bool has_inited = false;
    bool has_last_imu_data_ = false;

    double acc_noise_;
    double gyro_noise_;
    double acc_bias_noise_;
    double gyro_bias_noise_;
    double encoder_position_noise_;
    double v_noise_;

    double encoder_velo = 0.0;
    double position_x = 0.0;
    double position_y = 0.0;
    double pose_yaw = 0.0;

};
}
