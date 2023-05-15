/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-12-21 16:42:30
 * @LastEditors: luo
 * @LastEditTime: 2022-01-12 16:37:22
 */
#include "kalman_filter_pkg/graph_optimizer/graph_optimizer_flow.hpp"

namespace KalmanFilter
{

GraphOptimizerFlow::GraphOptimizerFlow(ros::NodeHandle& nh)
:nh_(nh)
{
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/synced_cloud", 100000);

    imu_raw_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/imu/data_xsens", 100000);
    imu_sys_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/synced_imu", 100000);

    lidar_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/matching_odom", 100000);

    fusion_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/fusion_odom_gp", "/map", "/fusion_odom_gp", 100);

    graph_optimizer_ptr_ = std::make_shared<GraphOptimizer>();

}

GraphOptimizerFlow::~GraphOptimizerFlow()
{


}

bool GraphOptimizerFlow::Run()
{
    if( !ReadData())
        return false;

    while(HasData())
    {
        if(!ValidData())
            continue;
        Update();

        PublishData();
    }

}

bool GraphOptimizerFlow::ReadData()
{
    // std::cout <<" ReadData" << std::endl;
    lidar_odom_sub_ptr_->ParseData(lidar_odom_data_buff_);

    imu_raw_sub_ptr_->ParseData(imu_raw_data_buff_);
    imu_sys_sub_ptr_->ParseData(imu_sys_data_buff_);

    // cloud_sub_ptr_->ParseData(cloud_deta_buff_);

    return true;


}

bool GraphOptimizerFlow::ValidData()
{
    // std::cout <<" ValidData" << std::endl;
    // current_cloud_data_ = cloud_deta_buff_.front();

    current_lidar_odom_data_ = lidar_odom_data_buff_.front();

    current_imu_data_ = imu_sys_data_buff_.front();

    double diff_imu_time = current_lidar_odom_data_.time - current_imu_data_.time;

    if(diff_imu_time < -0.05)
    {
        lidar_odom_data_buff_.pop_front();
        return false;
    }

    if(diff_imu_time > 0.05)
    {
        imu_sys_data_buff_.pop_front();
        return false;
    }

    lidar_odom_data_buff_.pop_front();
    imu_sys_data_buff_.pop_front();

    return true;


}

bool GraphOptimizerFlow::HasData()
{
    if(lidar_odom_data_buff_.empty() || imu_sys_data_buff_.empty() )
    {
        return false;
    }

    return true;

}

bool GraphOptimizerFlow::UpdateIMUPreIntegration()
{
    std::cout <<" UpdateIMUPreIntegration" << std::endl;
    
    while(!imu_raw_data_buff_.empty() 
            && imu_raw_data_buff_.front().time < current_imu_data_.time
            && graph_optimizer_ptr_->UpdateIMUPreIntrgration(imu_raw_data_buff_.front()))
    {
        imu_raw_data_buff_.pop_front();
    }

    return true;

}

bool GraphOptimizerFlow::Update()
{
    std::cout <<" Update" << std::endl;
    static bool odometry_inited = false;

    static Eigen::Matrix4f odom_init_pose = Eigen::Matrix4f::Identity();

    if(!odometry_inited)
    {
        odom_init_pose = current_lidar_odom_data_.pose;

        odometry_inited = true;
    }

    UpdateIMUPreIntegration();
    
    std::cout <<" Update 11" << std::endl;
    
    return graph_optimizer_ptr_->Update(current_lidar_odom_data_, current_imu_data_);

}


bool GraphOptimizerFlow::PublishData()
{
        // std::cout <<" PublishData 00" << std::endl;
    // fusion_odom_pub_ptr_->Publish(current_lidar_odom_data_.pose, current_lidar_odom_data_.time);
    if(graph_optimizer_ptr_->HasNewKeyFrame())
    {
        KeyFrame key_frame;
    }
    
    // std::cout <<" PublishData 11" << std::endl;
    if(graph_optimizer_ptr_->HasNewOptimized())
    {
        KeyFrame key_frame;
        graph_optimizer_ptr_->GetLatestOptimizedOdometry(key_frame);
        fusion_odom_pub_ptr_->Publish(key_frame.pose, key_frame.time);
    }
    // std::cout <<" PublishData 22" << std::endl;
    return true;
}


}