//
// Created by wchen on 2019/12/3.
//

#ifndef SRC_ESTIMATORNODELET_H
#define SRC_ESTIMATORNODELET_H

// C++ headers
#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <iomanip>
// ROS
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "gp_lio/utility/Utility.h"
#include "gp_lio/estimator/Estimator.h"
#include "gp_lio/utility/tic_toc.h"

namespace gp_lio
{

class EstimatorNodelet //: public nodelet::Nodelet
{
public:
//    virtual void onInit();
    EstimatorNodelet();
private:
    // initialize the variables in the class
    void InitializeVariables();

    // The measurements are processed and optimized here.
    void Run();

    // Store in the buffer and Propagate states in imu rate
    void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg);

    // Store in the buffer
    void CloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);

    // Propagate states in imu's rate
    void ImuPropagation(const sensor_msgs::ImuConstPtr &imu_msg);

    // Group imu measurements and lidar measurements:
    // make sure there are complete imu measurements between two cloud measurements
    MeasurementsGroupType GroupMeasurements();

    // Update states in LiDAR's rate after optimization
    void UpdateStates(State &state);

    void Command();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_imu_;
    ros::Subscriber sub_cloud_;

    // data buffer
    std::queue<sensor_msgs::ImuConstPtr> imu_buffer_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_buffer_;
    std::queue<sensor_msgs::PointCloud2ConstPtr> cloud_feature_buffer_;

    // data buffer shared in different threads
    std::condition_variable buffer_con_;
    std::mutex buffer_mutex_;

    // temporary states: [Qua, position, linear_velocity, angular_velocity, bias], from (imu_k} to {world}
    State tmp_state_;
    Eigen::Vector3d tmp_gravity_;
    std::mutex state_mutex_;

    // some useful variables for imu processing
    double last_imu_time_;
    Eigen::Vector3d last_imu_acc_;
    Eigen::Vector3d last_imu_gyr_;
    bool first_imu_;

    Estimator estimator_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace gp_lio

#endif //SRC_ESTIMATORNODELET_H
