/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-01 17:09:26
 * @LastEditors: luo
 * @LastEditTime: 2021-12-25 19:26:58
 */
#include "kalman_filter_pkg/front_end/front_end.hpp"
#include "glog/logging.h"

namespace KalmanFilter
{

FrontEnd::FrontEnd()
:local_map_ptr_(new CloudData::CloudPointT()),
global_map_ptr_(new CloudData::CloudPointT()),
result_cloud_ptr_(new CloudData::CloudPointT())
{
    InitWithConfig();
}

FrontEnd::~FrontEnd()
{

}

bool FrontEnd::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node)
{
    std::string registration_method = config_node["registration_method"].as<std::string>();
    std::cout << "Point Cloud Registration Method: " << registration_method << std::endl;

    if(registration_method == "NDT")
    {
        registration_ptr = std::make_shared<NDTRegistration>(config_node[registration_method]);
    }else if(registration_method == "ICP")
    {
        registration_ptr = std::make_shared<ICPRegistration> (config_node[registration_method]);
    }else 
    {
        return false;
    }

    return true;
    
}

bool FrontEnd::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node)
{
    std::string filter_mothod = config_node[filter_user + "_filter"].as<std::string>();
    std::cout << "Point Cloud Filter Method: " << filter_mothod << std::endl;

    if(filter_mothod == "voxel_filter")
    {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_mothod][filter_user]);
    
    }else 
    {
        std::cout << "Point Cloud Filter method " << filter_mothod << " Not Found!" << std::endl;
        return false;
    }

    return true;
    
}

bool FrontEnd::InitDataPath(YAML::Node& config_node)
{
    data_path_ =  config_node["data_path"].as<std::string>();
    if(data_path_ == "./")
    {
        data_path_ = data_path_;
    }
    std::cout <<" data_path = " << data_path_ << std::endl;

    data_path_ += "/slam_data";

    if(boost::filesystem::is_directory(data_path_))
    {
        boost::filesystem::remove_all(data_path_);
    }

    boost::filesystem::create_directory(data_path_);
    if(!boost::filesystem::is_directory(data_path_))
    {
        std::cout << "cannot create directory" << data_path_ << "! " << std::endl;
    }else 
    {
        std::cout << "Point Cloud Map output Path: " << data_path_ << std::endl;
    }


    std::string key_frame_path = data_path_ + "/key_frames";
    boost::filesystem::create_directory(data_path_ + "/key_frames");
    if(!boost::filesystem::is_directory(key_frame_path))
    {
        std::cout << "Cannot creat directory" << key_frame_path << std::endl;
        return false;
    }else 
    {
        std::cout << "key frames output Path: " << key_frame_path << std::endl << std::endl;
    }
    
    return true;
}

bool FrontEnd::InitWithConfig()
{
    std::string config_path = "/home/luo/work/filter_ws/src/kalman_filter_pkg/config/front_end.yaml";
    YAML::Node config_node = YAML::LoadFile(config_path);
    
    InitDataPath(config_node);
    InitRegistration(registration_ptr_ ,config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", frame_filter_ptr_, config_node);
    // InitFilter("display", display_filter_ptr_, config_node);

    
}

bool FrontEnd::SetInitPose(const Eigen::Matrix4f& init_pose)
{
    init_pose_ = init_pose;

    return false;
    
}

bool FrontEnd::Update(CloudData& cloud_data, Eigen::Matrix4f& cloud_pose)
{
    std::cout << "Update 00" << std::endl;

    current_frame_.cloud_data.time = cloud_data.time;

    // CloudData::CloudPointT input_cloud, output_cloud;
    // input_cloud = *cloud_data.cloud_ptr;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr, *current_frame_.cloud_data.cloud_ptr, indices);

     std::cout << "Update 00-1" << std::endl;

    // CloudData::CloudPointTPtr cloud_ptr = cloud_data.cloud_ptr;
    CloudData::CloudPointTPtr filtered_cloud_ptr(new CloudData::CloudPointT());
    frame_filter_ptr_->Filter(cloud_data.cloud_ptr, filtered_cloud_ptr);
    // current_frame_.cloud_data.cloud_ptr = filter_cloud_ptr;

    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f lase_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    static Eigen::Matrix4f last_key_frame_pose = init_pose_;

    std::cout << "Update 11" << std::endl;

    if(local_map_frames_.size() == 0)
    {
        current_frame_.pose = init_pose_;
        UpdateWithNewFrame(current_frame_);
        cloud_pose = current_frame_.pose;
        return true;
    }

    std::cout << "Update 22 filtered_cloud_ptr = " << filtered_cloud_ptr->size() << std::endl;

    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr_, current_frame_.pose);
    cloud_pose = current_frame_.pose;
    std::cout << "Update 22-3" << std::endl;
    step_pose = lase_pose.inverse() * current_frame_.pose;
    predict_pose = current_frame_.pose * step_pose;
    lase_pose = current_frame_.pose;

    std::cout << "Update 33" << std::endl;

    if(fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) + 
        fabs(last_key_frame_pose(1, 3) - current_frame_.pose(1, 3)) +
        fabs(last_key_frame_pose(2, 3) - current_frame_.pose(2, 3)) > key_frame_distance_)
    {
        UpdateWithNewFrame(current_frame_);
        last_key_frame_pose = current_frame_.pose;
    }

    std::cout << "Update 44" << std::endl;

    return true;
    
}

bool FrontEnd::UpdateWithNewFrame(const Frame& new_key_frame)
{
    std::string file_path = data_path_ + "/key_frames/key_frame_" + std::to_string(global_map_frames_.size()) + ".pcd";
    pcl::io::savePCDFileBinary(file_path, *new_key_frame.cloud_data.cloud_ptr);

    Frame key_frame = new_key_frame;
    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CloudPointT(*new_key_frame.cloud_data.cloud_ptr));
    CloudData::CloudPointTPtr transformed_cloud_ptr(new CloudData::CloudPointT());

    local_map_frames_.push_back(key_frame);
    while(local_map_frames_.size() > static_cast<size_t>(local_frame_num_))
    {
        local_map_frames_.pop_front();
    }

    local_map_ptr_.reset(new CloudData::CloudPointT());
    for(size_t i = 0; i < local_map_frames_.size(); ++i)
    {
        pcl::transformPointCloud(*local_map_frames_.at(i).cloud_data.cloud_ptr, *transformed_cloud_ptr, local_map_frames_.at(i).pose);

        *local_map_ptr_ += *transformed_cloud_ptr;
    }

    has_new_local_map_ = true;

    if(local_map_frames_.size() < 10)
    {
        registration_ptr_->SetInputTarget(local_map_ptr_);
    }else 
    {
        CloudData::CloudPointTPtr filtered_local_map_ptr(new CloudData::CloudPointT());
        local_map_filter_ptr_->Filter(local_map_ptr_, filtered_local_map_ptr);
        registration_ptr_->SetInputTarget(filtered_local_map_ptr);
    }

    key_frame.cloud_data.cloud_ptr.reset(new CloudData::CloudPointT());
    global_map_frames_.push_back(key_frame);

    return true;
    
}

    
}