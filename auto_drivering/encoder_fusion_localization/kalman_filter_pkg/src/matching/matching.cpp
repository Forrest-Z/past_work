/*
 * @Descripttion: 
 * @version: 
 * @Author: luo
 * @Date: 2021-11-09 16:53:19
 * @LastEditors: luo
 * @LastEditTime: 2022-01-13 15:39:31
 */
#include "kalman_filter_pkg/matching/matching.hpp"

namespace KalmanFilter
{

Matching::Matching()
:global_map_ptr_(new CloudData::CloudPointT()),
local_map_ptr_(new CloudData::CloudPointT()),
current_scan_ptr_(new CloudData::CloudPointT())
{

    InitWithConfig();

    InitGlobalMap();

    ResetLocalMap(0.0, 0.0, 0.0);

}


Matching::~Matching()
{


}

bool Matching::InitWithConfig()
{
    std::string config_file_path = "/home/luo/work/filter_ws/src/kalman_filter_pkg/config/matching.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);

    std::cout<<"***************地图定位初始化********************" << std::endl;
    InitDataPath(config_node);
    InitRegistration(registration_ptr_, config_node);
    InitFilter("global_map", global_map_filter_ptr_, config_node);
    InitFilter("local_map", local_map_filter_ptr_, config_node);
    InitFilter("frame", cloud_filter_ptr_, config_node);
    InitBoxFilter(config_node);


}

bool Matching::InitDataPath(const YAML::Node& config_node)
{
    
    map_path_ = config_node["map_path"].as<std::string>();

    return true;

}

bool Matching::InitRegistration(std::shared_ptr<RegistrationInterface>& registration_ptr, const YAML::Node& config_node)
{
    std::string matching_method = config_node["registration_method"].as<std::string>();
    std::cout <<"匹配方法为: " << matching_method << std::endl;

    if(matching_method == "ICP")
    {
        std::cout<< "ICP" << std::endl;
        registration_ptr = std::make_shared<ICPRegistration>(config_node[matching_method]);
        global_registration_ptr_ = std::make_shared<ICPRegistration>(config_node[matching_method]);

    }else if(matching_method == "NDT")
    {
        std::cout<< "NDT" << std::endl;
        registration_ptr = std::make_shared<NDTRegistration>(config_node[matching_method]);
        global_registration_ptr_ = std::make_shared<NDTRegistration>(config_node[matching_method]);

    }else 
    {
        std::cout<< "未知的匹配方法" << std::endl;

        return false;
    }

    return true;

}

bool Matching::InitFilter(std::string filter_user, std::shared_ptr<CloudFilterInterface>& filter_ptr, const YAML::Node& config_node)
{
    std::string filter_method = config_node[filter_user + "_filter"].as<std::string>();

    std::cout << "filter_user: " << filter_user << std::endl;
    std::cout << "filter_method: " << filter_method << std::endl;

    if(filter_method == "voxel_filter")
    {
        filter_ptr = std::make_shared<VoxelFilter>(config_node[filter_method][filter_user]);
    }else 
    {
        std::cout << "未知的滤波方法" << std::endl;
        return false;

    }

    return true;
}

bool Matching::InitBoxFilter(const YAML::Node& config_node)
{
    box_filter_ptr_ = std::make_shared<BoxFilter>(config_node);

    return true;

}



bool Matching::SetInitPose(const Eigen::Matrix4f& init_pose)
{
    std::cout << "Matching SetInitPose 00" << std::endl;
    init_pose_ = init_pose;

    ResetLocalMap(init_pose(0, 3), init_pose(1, 3), init_pose(2, 3));

    has_inited_ = true;
    std::cout << "Matching SetInitPose 11" << std::endl;
    return true;

}

bool Matching::InitKeyFrames(const YAML::Node& config_node)
{
    

}

bool Matching::LoadMap(CloudData::CloudPointTPtr& g_map, std::string map_path)
{
    std::string map_save_dir_ = map_path + "/map";
    std::string pose_graph_file = map_save_dir_ + "/pose_graph.txt";
    std::string surf_cloud = "_surf_cloud.pcd";
    std::string corner_cloud = "_corner_cloud.pcd";
    std::FILE* ifile_pg;
    ifile_pg = std::fopen(pose_graph_file.c_str(), "r");
    if (ifile_pg == NULL) {
        std::cout << "! Open [ " << pose_graph_file << " ] failed ..." << std::endl;
        return false;
    }

    global_KFs_.clear();
    KeyFramePoints tmp_kf;
    int i;
    double t, px, py, pz, qx, qy, qz, qw;
    while(std::fscanf(ifile_pg,"%d %lf %lf %lf %lf %lf %lf %lf %lf", &i, &t, &px, &py, &pz, &qx, &qy, &qz, &qw) !=EOF){
        tmp_kf.timestamp_ = t;
        tmp_kf.position_.x() = px;
        tmp_kf.position_.y() = py;
        tmp_kf.position_.z() = pz;
        tmp_kf.quaternion_.x() = qx;
        tmp_kf.quaternion_.y() = qy;
        tmp_kf.quaternion_.z() = qz;
        tmp_kf.quaternion_.w() = qw;
        pcl::io::loadPCDFile(map_save_dir_ + "/" + std::to_string(i) + surf_cloud, tmp_kf.surf_cloud_);
        pcl::io::loadPCDFile(map_save_dir_ + "/" + std::to_string(i) + corner_cloud, tmp_kf.corner_cloud_);

        *g_map += tmp_kf.GetTansformedCloud();
        global_KFs_.push_back(tmp_kf);
    }
    std::fclose(ifile_pg);
    std::cout << "load pointcloud finished ... "<<std::endl;

    
}

bool Matching::InitGlobalMap()
{
    // pcl::io::loadPCDFile(map_path_, *global_map_ptr_);
    LoadMap(global_map_ptr_, map_path_);

    std::cout << "load global map size = " << global_map_ptr_->points.size() << std::endl;

    CloudData::CloudPointTPtr g_map;
    local_map_filter_ptr_->Filter(global_map_ptr_, global_map_ptr_);
    std::cout << "filter global map size = " << global_map_ptr_->points.size() << std::endl;
    
    has_new_global_map_ = true;

    return true;
}

bool Matching::ResetLocalMap(float x, float y, float z)
{
    // std::cout<< "ResetLocalMap" << std::endl;
    std::vector<float> origin = {x, y, z};
    box_filter_ptr_->SetOrigin(origin);
    // std::cout<< "ResetLocalMap 00" << std::endl;
    box_filter_ptr_->Filter(global_map_ptr_, local_map_ptr_);

    // CloudData::CloudPointTPtr local_map = local_map_ptr_;
    // std::cout<< "ResetLocalMap 11 = " << local_map_ptr_->points.size() << std::endl;
    registration_ptr_->SetInputTarget(local_map_ptr_);

    // std::cout<< "ResetLocalMap 22" << std::endl;
    has_new_local_map_ = true;

    // std::vector<float> edge = box_filter_ptr_->GetEdge();

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    std::cout << "new local map:" << edge.at(0) << ","
                                  << edge.at(1) << ","
                                  << edge.at(2) << ","
                                  << edge.at(3) << ","
                                  << edge.at(4) << ","
                                  << edge.at(5) << std::endl << std::endl;

    return true;


}

bool Matching::Update(CloudData& cur_cloud_data, Eigen::Matrix4f& cur_pose)
{
    // std::cout << "Update 00" << std::endl;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cur_cloud_data.cloud_ptr, *cur_cloud_data.cloud_ptr, indices);

    CloudData::CloudPointTPtr filtered_cloud_ptr(new CloudData::CloudPointT());
    cloud_filter_ptr_->Filter(cur_cloud_data.cloud_ptr, filtered_cloud_ptr);
    // std::cout << "Update 11" << std::endl;
    static Eigen::Matrix4f step_pose = Eigen::Matrix4f::Identity();
    static Eigen::Matrix4f last_pose = init_pose_;
    static Eigen::Matrix4f predict_pose = init_pose_;
    // std::cout << "Update 22" << std::endl;
    if(!has_inited_)
    {
        // static int esti_count = 0;
        SetInitPose(init_pose_);
        has_inited_ = true;
    }
    // std::cout << "Update 33" << std::endl;
    CloudData::CloudPointTPtr result_cloud_ptr(new CloudData::CloudPointT());
    registration_ptr_->ScanMatch(filtered_cloud_ptr, predict_pose, result_cloud_ptr, cur_pose);
    pcl::transformPointCloud(*cur_cloud_data.cloud_ptr, *current_scan_ptr_, cur_pose);
    // std::cout << "Update 44" << std::endl;
    step_pose = last_pose.inverse() * cur_pose;
    predict_pose = cur_pose * step_pose;
    last_pose = cur_pose;

    // std::cout << " current pose = " << cur_pose << std::endl;
    std::cout << " ~~~~~~~~~~~~~~Matching::Update~~~~pose~~~~~~~~~~ " << std::endl;

    std::vector<float> edge = box_filter_ptr_->GetEdge();
    for(int i = 0; i < 3; i++)
    {
        if(fabs(cur_pose(i, 3) - edge.at(2 * i)) > 10.0 &&
            fabs(cur_pose(i, 3) - edge.at(2 * i + 1)) > 10.0 )
            continue;
        ResetLocalMap(cur_pose(0, 3), cur_pose(1, 3), cur_pose(2, 3));
        break;
    }

    return true;

}

void Matching::GetGlobalMap(CloudData::CloudPointTPtr& global_map)
{
    global_map_filter_ptr_->Filter(global_map_ptr_, global_map);
    has_new_global_map_ = true;

}

CloudData::CloudPointTPtr& Matching::GetLocalMap()
{
    return local_map_ptr_;

}

CloudData::CloudPointTPtr& Matching::GetCurrentScan()
{
    return current_scan_ptr_;

}


bool Matching::HasNewGlobalMap()
{
    return has_new_global_map_;
}

bool Matching::HasNewLocalMap()
{
    return has_new_local_map_;
}

bool Matching::HasInited()
{

    return has_inited_;

}

 
}