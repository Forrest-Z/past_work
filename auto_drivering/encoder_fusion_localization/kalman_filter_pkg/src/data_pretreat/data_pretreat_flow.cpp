/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-05 09:20:49
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 16:31:55
 */
#include "kalman_filter_pkg/data_pretreat/data_pretreat_flow.hpp"


namespace KalmanFilter
{

DataPrestreatFlow::DataPrestreatFlow(ros::NodeHandle& nh)
:nh_(nh)
{
    // cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/transfered__points", 100000);
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/velodyne_points", 100000);
    imu_sub_ptr_ = std::make_shared<IMUSubscriber>(nh_, "/imu/data_xsens", 100000);
    odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/localization/li_odom", 100000);
    encoder_odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/encoder_odom", 100000);

    // lidar_to_imu_ptr_ = std::make_shared<TFListener>(nh_, "/imu_link", "/velo_link");

    cloud_pub_ptr_ = std::make_shared<CloudPublisher>(nh_, "/synced_cloud", "/velo_link", 100);
    imu_pub_ptr_ = std::make_shared<IMUPublisher>(nh_, "/synced_imu", "/imu_link", 100);
    odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/synced_odom", "/map", "/odom_link", 100);
    encoder_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/synced_encoder_odom", "/map", "/encoder_link", 100);

}
DataPrestreatFlow::~DataPrestreatFlow()
{


}

bool DataPrestreatFlow::Run()
{
    if(!ReadData())
        return false;

    if(!InitCalibration())
        return false;
    
    if(!InitGNSS())
        return false;

    while(HasData())
    {
        if(!ValiedData())
            continue;
        
        TransformData();

        PublishData();
    }

    return true;
    
}

bool DataPrestreatFlow::ReadData()
{
    cloud_sub_ptr_->ParseData(cloud_data_buff_);

    // std::cout << "cloud_data_buff size = " << cloud_data_buff_.size() << std::endl;
    if(cloud_data_buff_.size() == 0)
        return false;

    double synced_time = cloud_data_buff_.front().time;
    
    static std::deque<IMUData> unsynced_imu_data;
    static std::deque<PoseData> unsynced_odom_data;


    imu_sub_ptr_->ParseData(unsynced_imu_data);
    odom_sub_ptr_->ParseData(odom_data_buff_);
    encoder_odom_sub_ptr_->ParseData(encoder_odom_data_buff_);

    // std::cout << "ReadData odom_data_buff_ size  = " << odom_data_buff_.size() << std::endl;

    bool imu_synced = IMUData::SyncData(unsynced_imu_data, imu_data_buff_, synced_time);
    // bool odom_synced = PoseData::SyncData(unsynced_odom_data, odom_data_buff_, synced_time);
    
    // std::cout << "ReadData 00 " << std::endl;
    // std::cout << "ReadData imu_synced = " << imu_synced << std::endl;
    // std::cout << "ReadData odom_synced = " << odom_synced << std::endl;
    static bool sensor_inited = false;
    if(!sensor_inited)
    {
        if(!imu_synced )
        {
            cloud_data_buff_.pop_front();
            return false;
        } 

        // std::cout << "ReadData 11 " << std::endl;
        sensor_inited = true;
    }

    // std::cout << "ReadData " << std::endl;
   
    return true;
}

bool DataPrestreatFlow::InitCalibration()
{
    static bool calibration = false;

    if(!calibration)
    {
        lidar_to_imu_ = Eigen::Matrix4f::Identity();
        // lidar_to_imu_(2, 3) = 0.3330773; 
        // lidar_to_imu_(0, 3) = -5.0; 

        calibration = true;
    }

    // std::cout << "InitCalibration " << std::endl;
    return calibration;

}

bool DataPrestreatFlow::InitGNSS()
{
    static bool gnss_inited = false;
    if(!gnss_inited)
    {
        odom_pose_ = Eigen::Matrix4f::Identity();

        gnss_inited = true;
    }

    // std::cout << "InitGNSS " << std::endl;
    return gnss_inited;

}

bool DataPrestreatFlow::HasData()
{
    // std::cout << "HasData 00" << std::endl;
    if(imu_data_buff_.size() == 0)
        return false;

    if(cloud_data_buff_.size() == 0)
        return false;

    // std::cout << "HasData 11" << std::endl;
    if( odom_data_buff_.size() == 0)
        return false;

    // std::cout << "HasData " << std::endl;
    if( encoder_odom_data_buff_.size() == 0)
        return false;

    std::cout << "HasData cloud_data_buff_ size = " << cloud_data_buff_.size() << std::endl;
    std::cout << "HasData imu_data_buff_ size = " << imu_data_buff_.size() << std::endl;
    std::cout << "HasData odom_data_buff_ size = " << odom_data_buff_.size() << std::endl;
    std::cout << "HasData encoder_odom_data_buff_ size = " << encoder_odom_data_buff_.size() << std::endl;
    return true;
}

bool DataPrestreatFlow::ValiedData()
{
    current_cloud_data_ = cloud_data_buff_.front();
    current_imu_data_ = imu_data_buff_.front();
    current_odom_data_ = odom_data_buff_.front();
    current_encoder_odom_data_ = encoder_odom_data_buff_.front();

    double diff_imu_time = current_cloud_data_.time - current_imu_data_.time;
    // double diff_odom_time = current_cloud_data_.time - current_odom_data_.time;
    double diff_encoder_time = current_cloud_data_.time - current_encoder_odom_data_.time;

    // double diff_imu_time = current_imu_data_.time - current_cloud_data_.time;
    // double diff_odom_time = current_odom_data_.time - current_cloud_data_.time;
    // std::cout << "ValiedData 00 " << std::endl;

    if(diff_imu_time < -0.05 && diff_encoder_time < -0.05)
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if(diff_imu_time > 0.05)
    {
        imu_data_buff_.pop_front();
        return false;
    }

    if(diff_encoder_time > 0.05)
    {
        encoder_odom_data_buff_.pop_front();
        return false;
    }


    cloud_data_buff_.pop_front();
    imu_data_buff_.pop_front();
    odom_data_buff_.pop_front();
    encoder_odom_data_buff_.pop_front();

    // std::cout << "ValiedData " << std::endl;

    return true;

}

bool DataPrestreatFlow::TransformData()
{
    // gnss_pose_ = Eigen::Matrix4f::Identity();

    // current_gnss_data_.UpdateXYZ();
    // gnss_pose_(0, 3) = current_gnss_data_.local_E;
    // gnss_pose_(1, 3) = current_gnss_data_.local_N;
    // gnss_pose_(2, 3) = current_gnss_data_.local_U;

    // gnss_pose_.block<3, 3>(0, 0) = current_imu_data_.GetOrientationMatrix();

    odom_pose_ = current_odom_data_.pose;
    // odom_pose_(0, 3) -= 6.0; 
    // odom_pose_(1, 3) -= 0.2; 

    odom_pose_ *= lidar_to_imu_;

    encoder_odom_pose_ = current_encoder_odom_data_.pose;


    // std::cout << "TransformData = " << std::endl;

    return true;
}

bool DataPrestreatFlow::PublishData()
{
    cloud_pub_ptr_->Publish(current_cloud_data_.cloud_ptr, current_cloud_data_.time);
    imu_pub_ptr_->Publish(current_imu_data_, current_cloud_data_.time);
    odom_pub_ptr_->Publish(odom_pose_, current_cloud_data_.time);
    encoder_odom_pub_ptr_->Publish(encoder_odom_pose_, current_cloud_data_.time);

    // std::cout << "PublishData cloud_data_buff_ size = " << current_cloud_data_.cloud_ptr->points.size() << std::endl;
    // std::cout << "PublishData imu_data_buff_ size = " << imu_data_buff_.size() << std::endl;
    // std::cout << "PublishData odom_data_buff_ size = " << odom_data_buff_.size() << std::endl;

    return true;

}




}