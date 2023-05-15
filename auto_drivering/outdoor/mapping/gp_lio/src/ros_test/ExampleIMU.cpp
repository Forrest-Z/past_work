//
// Created by wchen on 2020/4/24.
//

#include <ros/ros.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <sensor_msgs/Imu.h>
#include "gp_lio/initial/StaticInitializer.h"
#include "gp_lio/utility/Utility.h"
#include "gp_lio/utility/tic_toc.h"

#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <math.h>

using namespace std;
using namespace gp_lio;

// data buffer
std::queue<sensor_msgs::ImuConstPtr> imu_buffer_;

// data buffer shared in different threads
std::condition_variable buffer_con_;
std::mutex buffer_mutex_;

// temporary states: [Qua, position, linear_velocity, angular_velocity, bias], from (imu_k} to {world}
gp_lio::State tmp_state_;
Eigen::Vector3d tmp_gravity_;
std::mutex state_mutex_;

// some useful variables for imu processing
double last_imu_time_;
Eigen::Vector3d last_imu_acc_;
Eigen::Vector3d last_imu_gyr_;
bool first_imu_ = true;

// initialization
StaticInitializer staticInitializer_;
bool initialized = false;
size_t initalize_size = 200;

// moving_detection
double moving_threshold = 0.0;
size_t moving_size = 50;

void ComputeMeanStdev(double &stdev, std::vector<double> &vec)
{

    double sum = std::accumulate(std::begin(vec), std::end(vec), 0.0);
    double mean = sum / double(vec.size());

    double accum = 0.0;
    std::for_each(std::begin(vec), std::end(vec), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });

    stdev = sqrt(accum / double(vec.size() - 1));
}

std::vector<ImuMeasurement> GroupDataforMoving(bool pop = true)
{

    std::vector<ImuMeasurement> imu_mea_v;
    double last_at = 0.0;

    auto imu_buffer_backup = imu_buffer_;

    while (!imu_buffer_.empty())
    {

        double dx = imu_buffer_.front()->linear_acceleration.x;
        double dy = imu_buffer_.front()->linear_acceleration.y;
        double dz = imu_buffer_.front()->linear_acceleration.z;
        Eigen::Vector3d linear_acceleration{dx, dy, dz};

        double rx = imu_buffer_.front()->angular_velocity.x;
        double ry = imu_buffer_.front()->angular_velocity.y;
        double rz = imu_buffer_.front()->angular_velocity.z;
        Eigen::Vector3d angular_velocity{rx, ry, rz};

        ImuMeasurement i_mea;

        if (first_imu_)
        {
            i_mea.setIMU(imu_buffer_.front()->header.stamp.toSec(),
                         0.0,
                         linear_acceleration,
                         angular_velocity);
            first_imu_ = false;
        }
        else
        {
            i_mea.setIMU(imu_buffer_.front()->header.stamp.toSec(),
                         imu_buffer_.front()->header.stamp.toSec() - last_at,
                         linear_acceleration,
                         angular_velocity);
        }
        last_at = i_mea.at_;
        imu_mea_v.emplace_back(i_mea);
        imu_buffer_.pop();
    }

    if (!pop)
    {
        imu_buffer_ = imu_buffer_backup;
    }

    return imu_mea_v;
};

bool MovingDetector(std::vector<ImuMeasurement> imu_measurements, bool force_check = false)
{

    std::vector<double> ax_vec, ay_vec, az_vec;
    ax_vec.resize(imu_measurements.size());
    ay_vec.resize(imu_measurements.size());
    az_vec.resize(imu_measurements.size());
    for (size_t i = 0; i < imu_measurements.size(); i++)
    {
        ax_vec[i] = imu_measurements[i].ddx_;
        ay_vec[i] = imu_measurements[i].ddy_;
        az_vec[i] = imu_measurements[i].ddz_;
    }

    double ax_stdev;
    double ay_stdev;
    double az_stdev;
    ComputeMeanStdev(ax_stdev, ax_vec);
    ComputeMeanStdev(ay_stdev, ay_vec);
    ComputeMeanStdev(az_stdev, az_vec);

    double value_squ = pow(ax_stdev, 4) + pow(ay_stdev, 4) + pow(az_stdev, 4);

    ROS_DEBUG("moving check value: %f - %f", value_squ, moving_threshold * 1.5);

    if (!initialized && !force_check)
    {
        moving_threshold = value_squ;
        return false;
    }
    else
    {
        if (value_squ <= moving_threshold * 1.5)
            return false;
        else
            return true;
    }
}

void ImuPropagation(const sensor_msgs::ImuConstPtr &imu_msg)
{

    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    Eigen::Vector3d linear_acceleration{dx, dy, dz};

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Eigen::Vector3d angular_velocity{rx, ry, rz};

    // moving check in 50 imus window
    if (imu_buffer_.size() < moving_size)
        return;
    else
    {

        // ROS_DEBUG("buffer size: %d", imu_buffer_.size());
        std::vector<ImuMeasurement> imus = GroupDataforMoving(false);
        imu_buffer_.pop();
        if (!MovingDetector(imus))
        {
            last_imu_acc_ = linear_acceleration;
            last_imu_gyr_ = angular_velocity;
            last_imu_time_ = imu_msg->header.stamp.toSec();
//            ROS_ERROR("Robot is not moving!!!");
            return;
        }
    }

    //    if (first_imu_)
    //    {
    //        first_imu_= false;
    //        last_imu_acc_ = linear_acceleration;
    //        last_imu_gyr_ = angular_velocity;
    //        return;
    //    }

    double current_time = imu_msg->header.stamp.toSec();
    double dt = current_time - last_imu_time_;

    // mid-point integration for states propagation
    Eigen::Vector3d mid_angular_velocity = 0.5 * (last_imu_gyr_ + angular_velocity) - tmp_state_.bias_g_;
    Eigen::Quaterniond updated_q = tmp_state_.quaternion_ * gp_lio::Utility::deltaQ(mid_angular_velocity * dt);

    Eigen::Vector3d last_acc_in_world = tmp_state_.quaternion_ * (last_imu_acc_ - tmp_state_.bias_a_) - tmp_gravity_;
    Eigen::Vector3d linear_acceleration_in_world = updated_q * (linear_acceleration - tmp_state_.bias_a_) - tmp_gravity_;
    Eigen::Vector3d mid_linear_acceleration = 0.5 * (last_acc_in_world + linear_acceleration_in_world);

    tmp_state_.timestamp_ = current_time;
    tmp_state_.quaternion_ = updated_q;
    tmp_state_.position_ = tmp_state_.position_ + dt * tmp_state_.linear_velocity_ + 0.5 * dt * dt * mid_linear_acceleration;
    tmp_state_.linear_velocity_ = tmp_state_.linear_velocity_ + dt * mid_linear_acceleration;
    tmp_state_.angular_velocity_ = angular_velocity - tmp_state_.bias_g_; // in body-frame

    ROS_DEBUG_STREAM("dt     " << dt);
    ROS_DEBUG_STREAM("mid_linear_acceleration     " << mid_linear_acceleration.transpose());
    ROS_DEBUG_STREAM("Pi  " << tmp_state_.position_.transpose());
    ROS_DEBUG_STREAM("Ri  " << Utility::R2ypr(tmp_state_.quaternion_.toRotationMatrix()).transpose());
    ROS_DEBUG_STREAM("vi  " << tmp_state_.linear_velocity_.transpose());
    ROS_DEBUG_STREAM("wi  " << tmp_state_.angular_velocity_.transpose());

    // save for next propagation
    last_imu_acc_ = linear_acceleration;
    last_imu_gyr_ = angular_velocity;
    last_imu_time_ = imu_msg->header.stamp.toSec();
}

void ImuCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{

    if (imu_msg->header.stamp.toSec() <= last_imu_time_)
    {
//        ROS_ERROR("imu message disordered!!!");
        return;
    }

    // store
    buffer_mutex_.lock();
    imu_buffer_.push(imu_msg);
    std::cout << imu_msg->linear_acceleration.x
              << " " << imu_msg->linear_acceleration.y
              << " " << imu_msg->linear_acceleration.z
              << " " << imu_msg->angular_velocity.x
              << " " << imu_msg->angular_velocity.y
              << " " << imu_msg->angular_velocity.z << std::endl;
    buffer_mutex_.unlock();
    buffer_con_.notify_one();

    // update the states in imu rate
    if (initialized)
    {
        std::lock_guard<std::mutex> lockGuard(state_mutex_);
        ImuPropagation(imu_msg);
        //TODO publish [tmp_p_, tmp_q_, tmp_linear_velocity_, tmp_angular_velocity_] in visulization class
    }
}

bool init()
{

    if (!initialized && imu_buffer_.size() < initalize_size)
        return false;

    TicToc t_ini;
    // prepare data
    std::vector<ImuMeasurement> imu_mea_v = GroupDataforMoving();

    // moving threshold
    std::vector<ImuMeasurement> imu_mea_sgm_1(imu_mea_v.begin(), imu_mea_v.begin() + moving_size * 2);
    MovingDetector(imu_mea_sgm_1);

    // check moving
    std::vector<ImuMeasurement> imu_mea_sgm_2(imu_mea_v.end() - moving_size * 2, imu_mea_v.end());
    if (MovingDetector(imu_mea_sgm_2, true))
    {
        ROS_ASSERT("Please stay static for initialization!!!!!");
        return false;
    }

    // initialize
    staticInitializer_.GetIMUData(imu_mea_v);

    tmp_state_.timestamp_ = imu_mea_v.back().at_;
    tmp_state_.bias_a_ = staticInitializer_.bias_a_;
    tmp_state_.bias_g_ = staticInitializer_.bias_g_;

    Eigen::Matrix3d rot_0 = tmp_state_.quaternion_.toRotationMatrix();
    // double yaw = Utility::R2ypr(staticInitializer_.gravity_aligned_rot_ * rot_0).x();
    // Eigen::Matrix3d rot_diff = Utility::ypr2R(Eigen::Vector3d{-yaw, 0, 0}) * rot_0; // yaw is unobservable
    // tmp_gravity_ = rot_diff * staticInitializer_.gravity_;
    // rot_0 = rot_diff * rot_0;
    // tmp_state_.quaternion_ = Eigen::Quaterniond(rot_0);
    // tmp_state_.position_  = rot_diff*tmp_state_.position_;
    // tmp_state_.linear_velocity_ = rot_diff*tmp_state_.linear_velocity_;
    // ROS_DEBUG_STREAM("rot_diff     " << Utility::R2ypr(rot_diff).transpose());

    tmp_state_.quaternion_ = Eigen::Quaterniond(staticInitializer_.gravity_aligned_rot_ * rot_0);
    tmp_state_.position_ = staticInitializer_.gravity_aligned_rot_ * tmp_state_.position_;
    tmp_state_.linear_velocity_ = staticInitializer_.gravity_aligned_rot_ * tmp_state_.linear_velocity_;
    tmp_gravity_ = staticInitializer_.gravity_aligned_rot_ * staticInitializer_.local_gravity_;
    ROS_DEBUG_STREAM("g0     " << tmp_gravity_.transpose());
    ROS_DEBUG_STREAM("p0     " << tmp_state_.position_.transpose());
    ROS_DEBUG_STREAM("v0     " << tmp_state_.linear_velocity_.transpose());
    ROS_DEBUG_STREAM("my R0  " << Utility::R2ypr(tmp_state_.quaternion_.toRotationMatrix()).transpose());

    last_imu_acc_ = imu_mea_v.back().getAcc();
    last_imu_gyr_ = imu_mea_v.back().getGyr();
    last_imu_time_ = imu_mea_v.back().at_;

    ROS_DEBUG("****************initialization spends %f", t_ini.toc());
    initialized = true;

    return true;
}

void run()
{

    std::unique_lock<std::mutex> locker(buffer_mutex_);
    while (!init())
    {
        buffer_con_.wait(locker);
    }
    locker.unlock();
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ExampleIMU");
//    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::NodeHandle node("~");

    tmp_gravity_.z() = 9.81;

    last_imu_time_ = 0;
    last_imu_gyr_.setZero();
    last_imu_acc_.setZero();

    ros::Subscriber sub_imu_ = node.subscribe("/imu/data_xsens", 2000, &ImuCallback, ros::TransportHints().tcpNoDelay());

    std::thread run_thread(run);

    ros::spin();

    return 0;
}
