//
// Created by wchen on 2020/4/24.
//

#ifndef GP_LIO_STATICINITIALIZER_H
#define GP_LIO_STATICINITIALIZER_H

#include <ros/ros.h>
#include <vector>
#include <Eigen/Core>
#include "gp_lio/utility/Utility.h"
#include "gp_lio/estimator/Parameters.h"

namespace gp_lio
{

class StaticInitializer
{

public:
    StaticInitializer();

    ~StaticInitializer();

    void GetIMUData(std::vector<ImuMeasurement> imu_vector);

    bool MovingDetector(std::vector<ImuMeasurement> imu_measurements, bool force_check = false);

    bool MovingDetector(std::vector<sensor_msgs::ImuConstPtr> imu_measurements);

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Eigen::Vector3d local_gravity_; // local
    Eigen::Vector3d bias_a_;
    Eigen::Vector3d bias_g_;

    double noise_std_a_;
    double noise_std_w_;

    Eigen::Matrix3d gravity_aligned_rot_; // rotate local to world

    // moving_detection
    double moving_threshold_;
    size_t moving_size_;
};
} // namespace gp_lio

#endif //GP_LIO_STATICINITIALIZER_H