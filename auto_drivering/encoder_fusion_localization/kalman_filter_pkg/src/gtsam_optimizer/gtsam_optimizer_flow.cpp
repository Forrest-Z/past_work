/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-30 14:38:25
 * @LastEditors: luo
 * @LastEditTime: 2022-01-17 17:10:40
 */
#include <iostream>
#include <mutex>
#include <vector>
#include <queue>
#include <map>
#include <array>
#include <string>
#include <thread>

#include "kalman_filter_pkg/gtsam_optimizer/gtsam_optimizer_flow.hpp"
#include <glog/logging.h>

namespace KalmanFilter
{

GtsamOptimizer::GtsamOptimizer(ros::NodeHandle& nh)
:nh_(nh)
{
    extrinsic_matrix_ = Eigen::Matrix4d::Identity();
    Eigen::Quaterniond exq(extrinsic_matrix_.block<3, 3>(0, 0));
    Eigen::Vector3d ext = extrinsic_matrix_.block<3, 1>(0, 3);

    lidar_to_imu_ = gtsam::Pose3(gtsam::Rot3(exq), gtsam::Point3(ext.x(), ext.y(), ext.z()));
    imu_to_lidar_ = lidar_to_imu_.inverse();

    lio_opt_initialized_ = false;

    sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_xsens", 100000, &GtsamOptimizer::ImuCallBack, this);
    sub_prior_odometry_ = nh_.subscribe<nav_msgs::Odometry>("/localization/odom", 100000, &GtsamOptimizer::PriorOdometryCallBack, this);

    pub_li_odom_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/localization/gtsam_fusion_odom", "/map", "/lidar", 10000);    
    pub_imu_odometry_ = nh_.advertise<nav_msgs::Odometry>("/imu_pre_odom", 2000);

    //imu
    last_imu_time_opt_ = -1;
    imu_acc_noise_ = 0.1;
    imu_gyro_noise_ = 0.03;
    imu_acc_bias_N_ = 0.02;
    imu_gyro_bias_N_ = 4.0e-6;
    imu_gravity_ = 9.81007;
    boost::shared_ptr<gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imu_gravity_);
    p->accelerometerCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imu_acc_noise_, 2);            //加速度的白噪声
    p->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imu_gyro_noise_, 2);               //陀螺仪的白噪声
    p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) * pow(1e-4, 2);                        //通过速度积分位置信息引起的噪声
    gtsam::imuBias::ConstantBias prior_imu_bias((gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());   //初始化imu偏置

    prior_pose_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished());
    prior_vel_noise_ = gtsam::noiseModel::Isotropic::Sigma(3, 1e-4);
    prior_bias_noise_ = gtsam::noiseModel::Isotropic::Sigma(6, 1e-4);
    correction_noise_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished());
    correction_noise_2_ = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished());
    noise_model_between_bias_ = (gtsam::Vector(6) << imu_acc_bias_N_, imu_acc_bias_N_, imu_acc_bias_N_, imu_gyro_bias_N_, imu_gyro_bias_N_, imu_gyro_bias_N_).finished();

    //根据上面的参数， 定义两个imu预积分器，一个用于imu信息处理线程， 一个用于优化线程；
    imu_intrgrator_imu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);
    imu_intrgrator_opt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias);

    // std::thread* command(new std::thread(&GtsamOptimizer::Command, this));
}

GtsamOptimizer::~GtsamOptimizer()
{

}

void GtsamOptimizer::Run()
{
    while(HasData())
    {
        if(!ValidData())
            continue;

        if(UpdatePreintegration())
        {
            PublishData();
        }
        
    }

}


bool GtsamOptimizer::HasData()
{
    return !(prior_odom_buffer_.empty() || imu_opt_buffer_.empty() || imu_odom_buffer_.empty());

}

bool GtsamOptimizer::ValidData()
{
    mutex_.lock();

    current_prior_odom_pose_ = prior_odom_buffer_.front();

    prior_odom_buffer_.pop_front();

    mutex_.unlock();

    return true;
}

bool GtsamOptimizer::UpdatePreintegration()
{
    if(imu_opt_buffer_.empty())
        return false;

    mutex_.lock();

    cur_prior_odom_time_ = current_prior_odom_pose_->header.stamp.toSec();

    float p_x = current_prior_odom_pose_->pose.pose.position.x;
    float p_y = current_prior_odom_pose_->pose.pose.position.y;
    float p_z = current_prior_odom_pose_->pose.pose.position.z;
    float r_x = current_prior_odom_pose_->pose.pose.orientation.x;
    float r_y = current_prior_odom_pose_->pose.pose.orientation.y;
    float r_z = current_prior_odom_pose_->pose.pose.orientation.z;
    float r_w = current_prior_odom_pose_->pose.pose.orientation.w;
    
    bool degenerate = (int) current_prior_odom_pose_->pose.covariance[0] == 1 ? true : false;

    gtsam::Pose3 lidar_prior_pose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z), gtsam::Point3(p_x, p_y, p_z));

    std::cout << "------UpdatePreintegration---22---------" << std::endl;

    // 0. initialize system
    if(lio_opt_initialized_ == false)
    {
        ResetOptimization(); //初始化isam2优化器以及非线性因子图

        while(!imu_opt_buffer_.empty())
        {
            if(imu_opt_buffer_.front()->header.stamp.toSec() < cur_prior_odom_time_ - delta_t_)
            {
                last_imu_time_opt_ = imu_opt_buffer_.front()->header.stamp.toSec();
                imu_opt_buffer_.pop_front();
            }
            else 
                break;
        }

        //initial pose
        prev_pose_ = lidar_prior_pose.compose(lidar_to_imu_);
        gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, prior_pose_noise_);
        graph_factor_li_.add(prior_pose);

        //initial velocity
        prev_vel_ = gtsam::Vector3(0, 0, 0);
        gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, prior_vel_noise_);
        graph_factor_li_.add(prior_vel);

        //initial bias
        prev_bias_ = gtsam::imuBias::ConstantBias();
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(B(0), prev_bias_, prior_bias_noise_);
        graph_factor_li_.add(prior_bias);

        //add values
        initial_estimate_valus_li_.insert(X(0), prev_pose_);
        initial_estimate_valus_li_.insert(V(0), prev_vel_);
        initial_estimate_valus_li_.insert(B(0), prev_bias_);

        //optimize once
        isam_li_.update(graph_factor_li_, initial_estimate_valus_li_);
        graph_factor_li_.resize(0);
        initial_estimate_valus_li_.clear();
        isam_current_estimate_li_ = isam_li_.calculateEstimate();

        imu_intrgrator_opt_->resetIntegrationAndSetBias(prev_bias_);

        key = 1;
        mutex_.unlock();
        lio_opt_initialized_ = true;

        std::cout << "----------------------initialized finished-----------------" << std::endl;

        
        return true;
    }
    std::cout << "------UpdatePreintegration---33---------" << std::endl;

    //当isam2规模太大时， 进行边缘化， 重置优化器和因子图
    if(key == 100)
    {
        std::cout << "-------key--100---00---------" << std::endl;
        //获取最新关键帧的协方差
        gtsam::noiseModel::Gaussian::shared_ptr updated_pose_noise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(X(key - 1)));
        gtsam::noiseModel::Gaussian::shared_ptr updated_vel_noise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(V(key - 1)));
        gtsam::noiseModel::Gaussian::shared_ptr updated_bias_noise = gtsam::noiseModel::Gaussian::Covariance(isam_li_.marginalCovariance(B(key - 1)));

        std::cout << "-------key--100---11---------" << std::endl;
        //重置isam2优化器和因子图
        ResetOptimization();

        //按最新关键帧的协方差将位姿、速度、偏置因子添加到因子图中
        //add pose
        gtsam::PriorFactor<gtsam::Pose3> prior_pose(X(0), prev_pose_, updated_pose_noise);
        graph_factor_li_.add(prior_pose);
        //add velocity
        gtsam::PriorFactor<gtsam::Vector3> prior_vel(V(0), prev_vel_, updated_vel_noise);
        graph_factor_li_.add(prior_vel);
        //add bias
        gtsam::PriorFactor<gtsam::imuBias::ConstantBias> prior_bias(B(0), prev_bias_, updated_bias_noise);
        graph_factor_li_.add(prior_bias);

        //add values
        //并用最新的关键帧的位姿、速度、偏置初始化对应的因子
        initial_estimate_valus_li_.insert(X(0), prev_pose_);
        initial_estimate_valus_li_.insert(V(0), prev_vel_);
        initial_estimate_valus_li_.insert(B(0), prev_bias_);

        //并将最新初始化的因子图跟新到重置的isam2优化器中
        isam_li_.update(graph_factor_li_, initial_estimate_valus_li_);
        graph_factor_li_.resize(0);
        initial_estimate_valus_li_.clear();
        isam_current_estimate_li_ = isam_li_.calculateEstimate();
        std::cout << "-------key--100---end---------" << std::endl;
        key = 1;
    }
    std::cout << "------UpdatePreintegration---44---------" << std::endl;
    while ( !imu_opt_buffer_.empty())
    {
        sensor_msgs::ImuConstPtr this_imu = imu_opt_buffer_.front();
        double imu_time = this_imu->header.stamp.toSec();

        if(imu_time < cur_prior_odom_time_ - delta_t_)
        {
            double dt = (last_imu_time_opt_ < 0) ? (1.0 / 200.0) : ( imu_time - last_imu_time_opt_);
            imu_intrgrator_imu_->integrateMeasurement(
                gtsam::Vector3(this_imu->linear_acceleration.x, this_imu->linear_acceleration.y, this_imu->linear_acceleration.z),
                gtsam::Vector3(this_imu->angular_velocity.x, this_imu->angular_velocity.y, this_imu->angular_velocity.z),
                dt
            );

            last_imu_time_opt_ = imu_time;
            imu_opt_buffer_.pop_front();
        }
        else 
            break;
    }

    std::cout << "------UpdatePreintegration---55---------" << std::endl;
    //将imu因子添加到因子图中
    const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imu_intrgrator_opt_);
    //该因子涉及的优化变量包括： 上一帧和当前帧的位姿和速度， 上一帧的偏置， 相邻两关键帧之间的预积分结果
    gtsam::ImuFactor imu_factor(X(key -1), V(key - 1), X(key), V(key), B(key-1), preint_imu);
    graph_factor_li_.add(imu_factor);
    std::cout << "------UpdatePreintegration---55----00-----" << std::endl;
    
    //将imu偏置因子添加到因子图中
    graph_factor_li_.add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                            gtsam::noiseModel::Diagonal::Sigmas(sqrt(imu_intrgrator_opt_->deltaTij()) * noise_model_between_bias_)));
        std::cout << "------UpdatePreintegration---55-----11----" << std::endl;

    //添加当前关键帧位姿因子
    gtsam::Pose3 cur_pose = lidar_prior_pose.compose(lidar_to_imu_);
    gtsam::PriorFactor<gtsam::Pose3> pose_factor(X(key), cur_pose, degenerate ? correction_noise_2_ : correction_noise_);
    graph_factor_li_.add(pose_factor);

    std::cout << "------UpdatePreintegration---55-----22----" << std::endl;
    //设置当前关键帧位姿因子、速度因子和偏置因子的初始值
    gtsam::NavState prop_state_ = imu_intrgrator_opt_->predict(prev_state_, prev_bias_);
    initial_estimate_valus_li_.insert(X(key), prop_state_.pose());
    initial_estimate_valus_li_.insert(V(key), prop_state_.v());
    initial_estimate_valus_li_.insert(B(key), prev_bias_);
    std::cout << "------UpdatePreintegration---55-----33---" << std::endl;

    //将最新关键帧相关的因子图跟新到isam2优化器中， 并进行优化
    isam_li_.update(graph_factor_li_, initial_estimate_valus_li_);
    isam_li_.update();
    graph_factor_li_.resize(0);
    initial_estimate_valus_li_.clear();
    std::cout << "------UpdatePreintegration---55----44-----" << std::endl;

    //获取当前关键帧的优化结果， 并将结果置为先前值；
    gtsam::Values result = isam_li_.calculateEstimate();
    isam_current_estimate_li_ = isam_li_.calculateEstimate();
    prev_pose_ = result.at<gtsam::Pose3>(X(key));
    prev_vel_ = result.at<gtsam::Vector3>(V(key));
    prev_state_ = gtsam::NavState(prev_pose_, prev_vel_);
    prev_bias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
    std::cout << "------UpdatePreintegration---66---------" << std::endl;
    //利用优化后的imu偏置信息重置imu预积分
    imu_intrgrator_opt_->resetIntegrationAndSetBias(prev_bias_);

    //对优化结果进行失败检测： 当速度和偏置太大时，则认为优化失败
    if(FailureDetection(prev_vel_, prev_bias_))
    {
        ResetParams();
        return false;
    }
    std::cout << "------UpdatePreintegration---77---------" << std::endl;
    //2. 优化后， 重新对imu里程计进行预积分
    //利用优化结果跟新prev状态
    prev_state_odom_ = prev_state_;
    prev_bias_odom_ = prev_bias_;

    double imu_time2 = imu_odom_buffer_.front()->header.stamp.toSec();
    double last_imu_time = -1;
    while (!imu_odom_buffer_.empty() && imu_time2 < cur_prior_odom_time_ - delta_t_)
    {
        last_imu_time = imu_time2 = imu_odom_buffer_.front()->header.stamp.toSec();
        imu_odom_buffer_.pop_front();
    }

    if(!imu_odom_buffer_.empty())
    {
        imu_intrgrator_imu_->resetIntegrationAndSetBias(prev_bias_odom_);

        for(int i = 0; i < (int)imu_odom_buffer_.size(); ++i)
        {
            sensor_msgs::ImuConstPtr this_imu = imu_odom_buffer_[i];
            double imu_time3 = this_imu->header.stamp.toSec();
            double dt = (last_imu_time < 0) ? (1.0 / 200.0) : (imu_time3 - last_imu_time);
            imu_intrgrator_imu_->integrateMeasurement(
                gtsam::Vector3(this_imu->linear_acceleration.x, this_imu->linear_acceleration.y, this_imu->linear_acceleration.z),
                gtsam::Vector3(this_imu->angular_velocity.x, this_imu->angular_velocity.y, this_imu->angular_velocity.z),
                dt
            );

            last_imu_time = imu_time3;
        }
    }
    std::cout << "------UpdatePreintegration---88---------" << std::endl;
    mutex_.unlock();

    ++key;
    done_first_opt_ = true;

    return true;


}

bool GtsamOptimizer::PublishData()
{
    unsigned size = isam_current_estimate_li_.size() / 3;

    if(!size)
    {
        return false;
    }

    gtsam::Pose3 li_pose = isam_current_estimate_li_.at<gtsam::Pose3>(X(size - 1)).compose(imu_to_lidar_);
    gtsam::Vector3 li_velo = isam_current_estimate_li_.at<gtsam::Vector3>(V(size - 1));

    li_velocity_.x() = li_velo.x();
    li_velocity_.y() = li_velo.y();
    li_velocity_.z() = li_velo.z();

    Eigen::Quaterniond q;
    q.x() = li_pose.rotation().toQuaternion().x();
    q.y() = li_pose.rotation().toQuaternion().y();
    q.z() = li_pose.rotation().toQuaternion().z();
    q.w() = li_pose.rotation().toQuaternion().w();

    Eigen::Matrix3f R = q.normalized().toRotationMatrix().cast<float>();
    Eigen::Vector3f t;
    t.x() = li_pose.translation().x();
    t.y() = li_pose.translation().y();
    t.z() = li_pose.translation().z();

    li_odometry_.block<3, 3>(0, 0) = R;
    li_odometry_.block<3, 1>(0, 3) = t;

    Eigen::Quaternionf tmp_q(li_odometry_.block<3, 3>(0, 0));
    Eigen::Vector3f tmp_p(li_odometry_.topRightCorner(3, 1));
    double tmp_t = cur_prior_odom_time_;
    std::array<float, 7> tmp_odom{tmp_p.x(), tmp_p.y(), tmp_p.z(),
                                    tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()};
    
    odom_li_.push_back(std::make_pair(tmp_t, tmp_odom));

    pub_li_odom_ptr_->Publish(li_odometry_, li_velocity_, cur_prior_odom_time_);

    return true;
}

bool GtsamOptimizer::FailureDetection(const gtsam::Vector3& vel_cur, const gtsam::imuBias::ConstantBias& bias_cur)
{
    Eigen::Vector3f vel(vel_cur.x(), vel_cur.y(), vel_cur.z());

    if(vel.norm() > 30)
    {
        std::cout << "***********large velocity, reset imu-pre!!!!!!**************vel************* = "  << vel.norm() << std::endl;
        
        return true;
    }

    Eigen::Vector3f ba(bias_cur.accelerometer().x(), bias_cur.accelerometer().y(), bias_cur.accelerometer().z());
    Eigen::Vector3f bg(bias_cur.gyroscope().x(), bias_cur.gyroscope().y(), bias_cur.gyroscope().z());

    if(ba.norm() > 1.0 || bg.norm() > 1.0)
    {
        std::cout << "***********large bias, reset imu-pre!!!!!!******************ba********* = " << ba.norm() << std::endl;
        
        return true;       
    }

    return false;

}

void GtsamOptimizer::ResetOptimization()
{
    gtsam::ISAM2Params opt_Parameters;
    opt_Parameters.relinearizeThreshold = 0.1;
    opt_Parameters.relinearizeSkip = 1;

    isam_li_ = gtsam::ISAM2(opt_Parameters);

    //重置初始化非线性因子图
    gtsam::NonlinearFactorGraph new_graph_factors;
    graph_factor_li_ = new_graph_factors;

    gtsam::Values new_graph_Values;
    initial_estimate_valus_li_ = new_graph_Values;

}

void GtsamOptimizer::ResetParams()
{
    last_imu_time_opt_ = -1;

    done_first_opt_ = false;

    lio_opt_initialized_ = false;

}


void GtsamOptimizer::ImuCallBack(const sensor_msgs::ImuConstPtr& imu_raw)
{
    mutex_.lock();
    
    imu_opt_buffer_.push_back(imu_raw);
    imu_odom_buffer_.push_back(imu_raw);

    mutex_.unlock();

    if(done_first_opt_ == false)
    {
        return;
    }

    double imu_time = imu_raw->header.stamp.toSec();
    double dt = (last_imu_time_imu_ < 0) ? (1.0 / 200.0):(imu_time - last_imu_time_imu_);
    last_imu_time_imu_ = imu_time;

    //记录imu的测量信息
    imu_intrgrator_imu_->integrateMeasurement(
        gtsam::Vector3(imu_raw->linear_acceleration.x, imu_raw->linear_acceleration.y, imu_raw->linear_acceleration.z),
        gtsam::Vector3(imu_raw->angular_velocity.x, imu_raw->angular_velocity.y, imu_raw->angular_velocity.z),
        dt);
    
    //predict odometry
    //利用上一时刻的imu里程计状态信息PVQ和偏置信息， 预积分当前时刻imu里程计状态信息PVQ
    gtsam::NavState current_state = imu_intrgrator_imu_->predict(prev_state_odom_, prev_bias_odom_);

    //publish odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = imu_raw->header.stamp;
    odometry.header.frame_id = "/map";
    odometry.child_frame_id = "/lidar";

    gtsam::Pose3 imu_pose = gtsam::Pose3(current_state.quaternion(), current_state.position());
    gtsam::Pose3 lidar_pose = imu_pose.compose(imu_to_lidar_);

    odometry.pose.pose.position.x = lidar_pose.translation().x();
    odometry.pose.pose.position.y = lidar_pose.translation().y();
    odometry.pose.pose.position.z = lidar_pose.translation().z();

    odometry.pose.pose.orientation.x = lidar_pose.rotation().toQuaternion().x();
    odometry.pose.pose.orientation.y = lidar_pose.rotation().toQuaternion().y();
    odometry.pose.pose.orientation.z = lidar_pose.rotation().toQuaternion().z();
    odometry.pose.pose.orientation.w = lidar_pose.rotation().toQuaternion().w();

    odometry.twist.twist.linear.x = current_state.velocity().x();
    odometry.twist.twist.linear.y = current_state.velocity().y();
    odometry.twist.twist.linear.z = current_state.velocity().z();

    odometry.twist.twist.angular.x = imu_raw->angular_velocity.x + prev_bias_odom_.gyroscope().x();
    odometry.twist.twist.angular.y = imu_raw->angular_velocity.y + prev_bias_odom_.gyroscope().y();
    odometry.twist.twist.angular.z = imu_raw->angular_velocity.z + prev_bias_odom_.gyroscope().z();

    pub_imu_odometry_.publish(odometry);

}

void GtsamOptimizer::PriorOdometryCallBack(const nav_msgs::OdometryConstPtr& odom_msg)
{
    mutex_.lock();

    prior_odom_buffer_.push_back(odom_msg);

    mutex_.unlock();

}

void GtsamOptimizer::Command()
{


}

}