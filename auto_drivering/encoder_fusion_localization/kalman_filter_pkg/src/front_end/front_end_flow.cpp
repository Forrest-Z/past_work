/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:11:13
 * @LastEditors: luo
 * @LastEditTime: 2021-12-27 10:56:17
 */

#include "kalman_filter_pkg/front_end/front_end_flow.hpp"


namespace KalmanFilter
{

FrontEndFlow::FrontEndFlow(ros::NodeHandle& nh)
: nh_(nh)
{
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/synced_imu", 10000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/synced_cloud", 10000);
    odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/synced_odom", 10000);

    lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh_, "/imu_link", "/velo_link");
    front_end_ptr_ = std::make_shared<FrontEnd>();

    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/laser_odom", "/map", "/lidar", 100);

    local_map_ptr_.reset(new CloudData::CloudPointT());
    global_map_ptr_.reset(new CloudData::CloudPointT());
    current_scan_ptr_.reset(new CloudData::CloudPointT());

}


FrontEndFlow::~FrontEndFlow()
{


}

bool FrontEndFlow::Run()
{

    // std::cout<< "Run" << std::endl;

    if(!ReadData())
        return false;

    while (HasData())
    {
        if(!ValidData())
            continue;
        
        if(UpdateLaserOdometry())
        {
            PublishData();
        }else 
        {
            std::cout << " UpdateLaserOdometry Failed" << std::endl;
        }

        
    }

    return true;
}

bool FrontEndFlow::ReadData()
{
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    std::cout << "  cloud_data_buff_ = " << cloud_data_buff_.front().cloud_ptr->size() << std::endl;

    static std::deque<IMUData> unsynced_imu;

    imu_sub_ptr_->ParseData(unsynced_imu);
    odom_sub_ptr_->ParseData(odom_data_buff_);

    if(cloud_data_buff_.size() == 0)
        return false;

    double cloud_time = cloud_data_buff_.front().time;

    bool valid_imu = IMUData::SyncData(unsynced_imu, imu_data_buff_, cloud_time);

    static bool sensor_inited = false;

    if(!sensor_inited)
    {
        if(!valid_imu )
        {
            cloud_data_buff_.pop_front();

            return false;
        }

        sensor_inited = true;
    }

    return true;

}


bool FrontEndFlow::HasData()
{
    if(cloud_data_buff_.size() == 0)
        return false;

    if(imu_data_buff_.size() == 0)
        return false;

    if(odom_data_buff_.size() == 0)
        return false;

    return true;

}

bool FrontEndFlow::ValidData()
{
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_odom_data_ = odom_data_buff_.front();

    double delta_time = current_cloud_data_.time - current_imu_data_.time;
    double delta_odom_time = current_cloud_data_.time - current_odom_data_.time;

    if(delta_time < -0.05 || delta_odom_time < -0.05)
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if(delta_time > 0.05)
    {
        imu_data_buff_.pop_front();

        return false;
    }

    if(delta_odom_time > 0.05)
    {
        odom_data_buff_.pop_front();

        return false;
    }

    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();

    return true;

}


bool FrontEndFlow::PublishData()
{
    laser_odom_pub_ptr_->Publish(laser_odometry_,current_cloud_data_.time);

    return true;

}

bool FrontEndFlow::UpdateLaserOdometry()
{
    // std::cout << "UpdateLaserOdometry 00" << std::endl;
    static bool front_end_pose_inited = false;

    if(!front_end_pose_inited)
    {
        front_end_pose_inited = true;

        front_end_ptr_->SetInitPose(current_odom_data_.pose);
    }

    // std::cout << "UpdateLaserOdometry 11" << std::endl;
    laser_odometry_ = Eigen::Matrix4f::Identity();
    // laser_odometry_ = current_odom_data_.pose;
    return front_end_ptr_->Update(current_cloud_data_, laser_odometry_);
  
    
}

}