/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-09 16:53:11
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 15:39:23
 */
#include "kalman_filter_pkg/matching/matching_flow.hpp"

namespace KalmanFilter
{

MatchingFlow::MatchingFlow(ros::NodeHandle& nh)
:nh_(nh)
{
    
    cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh_, "/synced_cloud", 100000);
    gnss_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh_, "/synced_odom", 100000);

    local_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/local_map", "/map", 100000);
    global_map_pub_ptr_ = std::make_shared<CloudPublisher>(nh, "/global_map", "/map", 100000);

    laser_odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh_, "/matching_odom", "/map", "lidar", 10000);

    matching_ptr_ = std::make_shared<Matching>();


}

MatchingFlow::~MatchingFlow()
{


}

bool MatchingFlow::Run()
{
    if(matching_ptr_->HasNewGlobalMap() && global_map_pub_ptr_->HasSubscribers())
    {
        CloudData::CloudPointTPtr global_map_ptr(new CloudData::CloudPointT());
        matching_ptr_->GetGlobalMap(global_map_ptr);
        global_map_pub_ptr_->Publish(global_map_ptr);
    }

    if(matching_ptr_->HasNewLocalMap() && local_map_pub_ptr_->HasSubscribers())
    {
        local_map_pub_ptr_->Publish(matching_ptr_->GetLocalMap());
    }

    ReadData();

    while(HasData())
    {
        if(!ValidData())
            continue;
        if(UpdateMatching())
        {
            PublishData();
        }
    }

    return true;
}

bool MatchingFlow::ReadData()
{
    cloud_sub_ptr_->ParseData(cloud_data_buff_);
    gnss_sub_ptr_->ParseData(gnss_data_buff_);
    
    return true;

}

bool MatchingFlow::HasData()
{
    if(cloud_data_buff_.size() == 0)
        return false;

    if(matching_ptr_->HasInited())
        return true;

    if(gnss_data_buff_.size() == 0)
        return false;
        
    return true;

}

bool MatchingFlow::ValidData()
{

    cur_cloud_ = cloud_data_buff_.front();

    if(matching_ptr_->HasInited())
    {
        cloud_data_buff_.pop_front();
        gnss_data_buff_.clear();

        return true;
    }

    cur_gnss_ = gnss_data_buff_.front();

    // std::cout << "cloud time = " << cur_cloud_.time << std::endl;
    // std::cout << "gnss time = " << cur_gnss_.time << std::endl;
    // std::cout << "gnss pose = " << cur_gnss_.pose << std::endl;


    double diff_time = cur_cloud_.time - cur_gnss_.time;
    if(diff_time < -0.05)
    {
        cloud_data_buff_.pop_front();
        return false;
    }

    if(diff_time > 0.05)
    {
        gnss_data_buff_.pop_front();
        return false;
    }
    

    cloud_data_buff_.pop_front();
    gnss_data_buff_.pop_front();

    return true;

}

bool MatchingFlow::UpdateMatching()
{

    // static bool matching_inited = false;
    if(!matching_ptr_->HasInited())
    {
        matching_ptr_->SetInitPose(cur_gnss_.pose);
        // matching_ptr_->SetInitPose(Eigen::Matrix4f::Identity());

        // matching_inited = true;
        return true;
    }

    cur_pose_ = cur_gnss_.pose;

    return matching_ptr_->Update(cur_cloud_, cur_pose_);

}

bool MatchingFlow::PublishData()
{
    laser_odom_pub_ptr_->Publish(cur_pose_, cur_cloud_.time);

    return true;
    
}
 
}